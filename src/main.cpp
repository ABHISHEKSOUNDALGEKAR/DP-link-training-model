// Test runner: directed scenarios + constrained-random regression.
//
// Each test builds a sink (with optional error injection) and a source, connects
// them through the AUX channel, attaches the monitor (checks + coverage), runs
// link training, and then scores the result:
//   - scoreboard: final (rate, lanes) must match the expected/reference result
//   - checker:    observed rule violations must match the expected set
//
// Usage:
//   ./dp_lt_model                       run all directed tests + 100 random seeds
//   ./dp_lt_model --list                list directed tests
//   ./dp_lt_model <test> [-v]           run one directed test (-v prints AUX log)
//   ./dp_lt_model --random <N> [--seed <S>] [-v]

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "aux_channel.h"
#include "link_monitor.h"
#include "sink_model.h"
#include "source_trainer.h"

using namespace dp;

struct Expect {
    bool success = true;
    bool use_reference = true;           // compute expected config with the reference model
    LinkRate rate = LinkRate::HBR3;
    uint8_t lanes = 4;
    std::set<std::string> rules;         // checker rules that MUST fire (empty = clean run)
};

struct TestCase {
    std::string name;
    std::string what;
    SinkConfig sink;
    SourceConfig src;
    Expect expect;
};

// ------------------------------------------------------------- reference model
// Predicts the configuration a correct source should end up with, from the channel
// description alone. Used as the scoreboard's "golden" answer.
static bool reference(const SinkConfig& sink, const SourceConfig& src, LinkRate& rate_out, uint8_t& lanes_out) {
    const uint8_t start_code = std::min(static_cast<uint8_t>(sink.max_rate), static_cast<uint8_t>(src.max_rate));
    uint8_t lanes = std::min(sink.max_lanes, src.max_lanes);
    lanes = lanes >= 4 ? 4 : lanes >= 2 ? 2 : 1;
    bool skip_first = sink.drop_cr_once_during_eq;

    for (; lanes >= 1; lanes /= 2) {
        LinkRate rate = static_cast<LinkRate>(start_code);
        while (true) {
            bool ok = true;
            for (int l = 0; l < lanes; ++l) {
                const LaneChannel c = sink.channel[rate_index(rate)][l];
                ok &= (c.cr_swing + c.eq_preemph <= MAX_LEVEL);
            }
            if (rate == LinkRate::HBR3 && sink.tps4_supported && !src.tps4_supported) ok = false;
            if (skip_first) {
                ok = false;  // CR is lost during EQ on the very first attempt -> fallback
                skip_first = false;
            }
            if (ok) {
                rate_out = rate;
                lanes_out = lanes;
                return true;
            }
            LinkRate lower;
            if (!next_lower_rate(rate, lower)) break;
            rate = lower;
        }
        if (lanes == 1) break;
    }
    return false;
}

// ------------------------------------------------------------- directed tests
static void set_all(SinkConfig& s, LinkRate r, uint8_t cr_swing, uint8_t eq_pe) {
    for (auto& lane : s.channel[rate_index(r)]) lane = {cr_swing, eq_pe};
}

static std::vector<TestCase> directed_tests() {
    std::vector<TestCase> t;

    { TestCase tc{"happy_hbr3_x4", "Clean channel: trains HBR3 x4 on the first try", {}, {}, {}};
      t.push_back(tc); }

    { TestCase tc{"cr_adjust_requests", "HBR3 needs VS2 for CR and PE1 for EQ: several adjust loops", {}, {}, {}};
      set_all(tc.sink, LinkRate::HBR3, 2, 1);
      t.push_back(tc); }

    { TestCase tc{"rate_fallback_lane2", "Lane 2 needs VS3+PE2 at HBR3 (impossible): falls back to HBR2", {}, {}, {}};
      tc.sink.channel[rate_index(LinkRate::HBR3)][2] = {3, 2};
      t.push_back(tc); }

    { TestCase tc{"lane_fallback", "Lane 3 can't train at any rate: drops to 2 lanes", {}, {}, {}};
      for (auto& per_rate : tc.sink.channel) per_rate[3] = {2, 2};
      t.push_back(tc); }

    { TestCase tc{"sink_hbr2_x2", "Sink only supports HBR2 x2: source must respect DPCD caps", {}, {}, {}};
      tc.sink.max_rate = LinkRate::HBR2;
      tc.sink.max_lanes = 2;
      t.push_back(tc); }

    { TestCase tc{"aux_defer_storm", "Sink DEFERs the first 5 requests: source retries and succeeds", {}, {}, {}};
      tc.sink.defer_next = 5;
      t.push_back(tc); }

    { TestCase tc{"aux_timeouts", "Sink ignores 2 requests (no reply): source times out and retries", {}, {}, {}};
      tc.sink.timeout_next = 2;
      t.push_back(tc); }

    { TestCase tc{"aux_retry_exhausted", "Sink DEFERs 9 times in a row: source gives up cleanly", {}, {}, {}};
      tc.sink.defer_next = 9;
      tc.expect.success = false;
      tc.expect.use_reference = false;
      t.push_back(tc); }

    { TestCase tc{"cr_lost_during_eq", "Lane 0 loses CR during EQ once: source falls back a rate", {}, {}, {}};
      tc.sink.drop_cr_once_during_eq = true;
      t.push_back(tc); }

    { TestCase tc{"slow_cdr_lock", "Sink withholds CR_DONE for 4 updates: CR needs 5 polls, still HBR3", {}, {}, {}};
      tc.sink.cr_hold_updates = 4;
      t.push_back(tc); }

    { TestCase tc{"eq_slow_interval", "Sink asks for a 4 ms EQ read interval: source must honor it", {}, {}, {}};
      tc.sink.aux_rd_interval = 1;
      set_all(tc.sink, LinkRate::HBR3, 1, 2);
      t.push_back(tc); }

    { TestCase tc{"single_lane_rbr", "Sink supports only RBR on 1 lane: smallest legal link", {}, {}, {}};
      tc.sink.max_rate = LinkRate::RBR;
      tc.sink.max_lanes = 1;
      t.push_back(tc); }

    { TestCase tc{"rbr_only_channel", "Lane 1 can't train above RBR: falls all the way to RBR x4", {}, {}, {}};
      for (LinkRate r : {LinkRate::HBR, LinkRate::HBR2, LinkRate::HBR3})
          tc.sink.channel[rate_index(r)][1] = {3, 1};
      t.push_back(tc); }

    // ---- negative tests: a buggy source; the checker MUST catch it ----
    { TestCase tc{"neg_illegal_drive", "BUGGY SOURCE starts at VS2+PE2: checker must flag LT-1", {}, {}, {}};
      tc.src.bug_start_with_high_drive = true;
      tc.expect.rules = {"LT-1"};
      t.push_back(tc); }

    { TestCase tc{"neg_no_read_interval", "BUGGY SOURCE polls status immediately: checker must flag LT-6", {}, {}, {}};
      tc.src.bug_skip_read_interval = true;
      tc.expect.rules = {"LT-6"};
      t.push_back(tc); }

    { TestCase tc{"neg_hbr3_without_tps4", "Source lacks TPS4 at HBR3: checker flags LT-5, link falls back", {}, {}, {}};
      tc.src.tps4_supported = false;
      tc.expect.rules = {"LT-5"};
      t.push_back(tc); }

    return t;
}

// ------------------------------------------------------------- constrained random
static TestCase random_test(uint32_t seed) {
    std::mt19937 rng(seed);
    auto pick = [&](int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng); };

    TestCase tc;
    tc.name = "random_seed_" + std::to_string(seed);
    tc.what = "constrained-random channel and AUX errors";
    // Bias toward the common case (HBR3 x4) while still reaching every rate and lane count.
    static const LinkRate kRates[] = {LinkRate::RBR, LinkRate::HBR, LinkRate::HBR2, LinkRate::HBR3};
    const int r = pick(0, 9);
    tc.sink.max_rate = r < 5 ? LinkRate::HBR3 : kRates[r % 4];
    const int l = pick(0, 7);
    tc.sink.max_lanes = l < 4 ? 4 : l < 6 ? 2 : 1;
    tc.sink.tps3_supported = pick(0, 3) != 0;
    // Higher rates are harder: requirements grow with the rate index.
    for (int ri = 0; ri < 4; ++ri)
        for (auto& lane : tc.sink.channel[ri])
            lane = {static_cast<uint8_t>(pick(0, std::min(3, ri))), static_cast<uint8_t>(pick(0, ri))};
    tc.sink.defer_next = pick(0, 4);
    tc.sink.timeout_next = pick(0, 2);
    tc.sink.drop_cr_once_during_eq = pick(0, 4) == 0;
    tc.sink.aux_rd_interval = pick(0, 5) == 0 ? 1 : 0;
    return tc;
}

// ------------------------------------------------------------- runner
static bool run(TestCase tc, bool verbose, Coverage& total_cov) {
    Coverage cov;
    define_link_training_coverage(cov);

    SinkModel sink(tc.sink);
    AuxChannel aux(sink);
    LinkMonitor monitor(cov);
    TransactionLogger logger;
    aux.add_observer(&monitor);
    if (verbose) aux.add_observer(&logger);

    if (verbose) std::printf("\n=== %s: %s\n", tc.name.c_str(), tc.what.c_str());
    SourceTrainer source(aux, tc.src);
    const TrainResult r = source.train();
    monitor.final_check(aux.now_us());

    // Scoreboard
    Expect e = tc.expect;
    if (e.use_reference) e.success = reference(tc.sink, tc.src, e.rate, e.lanes);
    std::string why;
    if (r.success != e.success) why = r.success ? "trained but expected failure" : "failed: " + r.fail_reason;
    else if (r.success && (r.rate != e.rate || r.lanes != e.lanes))
        why = std::string("got ") + to_string(r.rate) + " x" + std::to_string(r.lanes) + ", expected " +
              to_string(e.rate) + " x" + std::to_string(e.lanes);

    // Checker
    std::set<std::string> fired;
    for (const auto& v : monitor.violations()) fired.insert(v.rule);
    if (why.empty() && fired != e.rules) {
        why = "checker rules fired {";
        for (const auto& f : fired) why += " " + f;
        why += " } expected {";
        for (const auto& f : e.rules) why += " " + f;
        why += " }";
    }

    const bool pass = why.empty();
    std::printf("%s  %-24s %-10s %6.2f ms  %s\n", pass ? "PASS" : "FAIL", tc.name.c_str(),
                r.success ? (std::string(to_string(r.rate)) + " x" + std::to_string(r.lanes)).c_str() : "no link",
                aux.now_us() / 1000.0, pass ? "" : why.c_str());
    if (verbose || !pass)
        for (const auto& v : monitor.violations())
            std::printf("      [%7llu us] %s: %s\n", static_cast<unsigned long long>(v.time_us), v.rule.c_str(),
                        v.message.c_str());

    // Only clean, passing runs contribute coverage (a failing test's coverage can't be trusted).
    if (pass && e.rules.empty()) total_cov.merge(cov);
    return pass;
}

int main(int argc, char** argv) {
    bool verbose = false, list = false;
    int random_n = -1;
    uint32_t seed = 1;
    std::string only;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "-v")) verbose = true;
        else if (!std::strcmp(argv[i], "--list")) list = true;
        else if (!std::strcmp(argv[i], "--random") && i + 1 < argc) random_n = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--seed") && i + 1 < argc) seed = static_cast<uint32_t>(std::atoi(argv[++i]));
        else only = argv[i];
    }

    const auto tests = directed_tests();
    if (list) {
        for (const auto& t : tests) std::printf("%-24s %s\n", t.name.c_str(), t.what.c_str());
        return 0;
    }

    Coverage total;
    define_link_training_coverage(total);
    int run_count = 0, passed = 0;

    auto go = [&](const TestCase& tc) {
        ++run_count;
        passed += run(tc, verbose, total);
    };

    if (!only.empty()) {
        for (const auto& t : tests) if (t.name == only) go(t);
        if (run_count == 0) {
            std::printf("unknown test '%s' (try --list)\n", only.c_str());
            return 2;
        }
    } else if (random_n >= 0) {
        for (int i = 0; i < random_n; ++i) go(random_test(seed + static_cast<uint32_t>(i)));
    } else {
        std::printf("Directed tests\n");
        for (const auto& t : tests) go(t);
        std::printf("\nConstrained-random regression (100 seeds)\n");
        for (uint32_t s = 1; s <= 100; ++s) go(random_test(s));
    }

    std::printf("\n%d/%d tests passed\n", passed, run_count);
    if (only.empty()) total.report();
    return passed == run_count ? 0 : 1;
}
