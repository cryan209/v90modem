// Two MICA boards on one line: A answers, B originates (dials), and each board's audio is the other's input.
// usage: mica_pair [--map FILE] [--seconds S] [--s29-a N] [--s29-b N] [--dial DIGITS] [--cap-a F] [--cap-b F]
//        env MICA_RX=1 prints each board's received-word statistics per second
#include "mica_board.h"
#include "mica_dte.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
using namespace mica;
constexpr size_t BLOCK = 160;
struct Side {
    const char *name;
    Board *board = nullptr;
    size_t cursor = 0;
    std::vector<uint8_t> mono;        // this board's output, lane 0
    size_t consumed = 0;
    std::vector<uint8_t> capture;
    int state = -1;
    size_t answered_at = 0;
    uint64_t words = 0, ones = 0;
    std::vector<uint16_t> all;
    std::vector<uint16_t> message;
};
int main(int argc, char **argv)
{
    std::string map = "mica-portware-map.bin", dial = "1", cap_a, cap_b;
    double seconds = 30; int s29a = 3, s29b = 3; bool originate_b = true;
    for(int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto v = [&]() { return std::string(argv[++i]); };
        if(a == "--map") map = v(); else if(a == "--seconds") seconds = std::stod(v());
        else if(a == "--s29-a") s29a = std::stoi(v()); else if(a == "--s29-b") s29b = std::stoi(v());
        else if(a == "--dial") dial = v(); else if(a == "--cap-a") cap_a = v(); else if(a == "--cap-b") cap_b = v();
        else if(a == "--b-answers") originate_b = false;
    }
    std::ifstream f(map, std::ios::binary);
    std::vector<uint8_t> image((std::istreambuf_iterator<char>(f)), {});
    Board A(image), B(image);
    Side sa{"A"}, sb{"B"}; sa.board = &A; sb.board = &B;
    Side *sides[2] = {&sa, &sb};
    for(Side *s : sides) { s->board->set_idle_skip(true); s->board->set_interrupts(true); s->board->boot(); }
    auto init_at = [&](Board &b, int n) {
        std::string t = "ATS29=" + std::to_string(n) + "\r";
        b.dte_write(0, reinterpret_cast<const uint8_t *>(t.data()), t.size());
        b.run(3000000);
        uint8_t sink[256]; while(b.dte_read(0, sink, sizeof sink) > 0) {}
    };
    init_at(A, s29a); init_at(B, s29b);
    if(const char *both = std::getenv("AT_BOTH")) {      // "ATS30=4800;ATS32=0"
        std::string text = both;
        size_t pos = 0;
        while(pos < text.size()) {
            size_t semi = text.find(';', pos);
            std::string cmd = text.substr(pos, semi == std::string::npos ? std::string::npos : semi - pos) + "\r";
            for(Board *b : {&A, &B}) {
                b->dte_write(0, reinterpret_cast<const uint8_t *>(cmd.data()), cmd.size());
                b->run(3000000);
                uint8_t sink[512]; while(b->dte_read(0, sink, sizeof sink) > 0) {}
            }
            if(semi == std::string::npos) break;
            pos = semi + 1;
        }
    }
    auto call_time = [&](Side &s) { return (s.board->dsp(0).g711_tx().size() - s.answered_at) / 16000.0; };
    for(Side *s : sides) {
        Side *sp = s;
        s->board->on_slice = [sp, &call_time] {
            while(sp->board->take_message(0, sp->message)) {
                if(std::getenv("MICA_MSG")) { std::fprintf(stderr, "%s msg:", sp->name); for(auto w : sp->message) std::fprintf(stderr, " %04x", w); std::fprintf(stderr, "\n"); }
                int now = channel_state(sp->message);
                if(now >= 0 && now != sp->state) {
                    std::fprintf(stderr, "%s: %s at %.2f s\n", sp->name, channel_state_name(now), call_time(*sp));
                    sp->state = now;
                }
            }
        };
        s->board->on_dsp_write = [sp](int dsp, uint32_t address, uint16_t value) {
            if(dsp == 0 && address >= 0xFEFFE98C && address < 0xFEFFE9FF) { sp->words++; sp->ones += __builtin_popcount(value); sp->all.push_back(value); }
        };
    }
    // A answers, as mica_line does.
    A.configure_call(0, false); A.run(300000); A.dsp_command(0, 0xA0, {0});
    // B originates and dials.
    B.configure_call(0, originate_b); B.run(1500000);
    if(originate_b && std::getenv("B_ANSWER_START")) { B.dsp_command(0, 0xA0, {0}); B.run(500000); }
    if(originate_b) {
        std::vector<uint16_t> words{uint16_t((dial.size() << 8) | 0x29)};
        for(char c : dial) words.push_back(uint16_t(c));
        if(!B.send(0, words, 1)) std::fprintf(stderr, "dial queue busy\n");
    } else B.dsp_command(0, 0xA0, {0});
    for(Side *s : sides) { s->cursor = s->board->dsp(0).g711_tx().size(); s->answered_at = s->cursor; }
    // Prime each with two idle blocks.
    std::vector<uint8_t> idle(BLOCK, 0xFF);
    auto feed = [&](Side &to, const uint8_t *blk) {
        uint8_t packed[BLOCK * 2];
        for(size_t i = 0; i < BLOCK; ++i) packed[2 * i] = packed[2 * i + 1] = blk[i];
        to.board->dsp(0).queue_g711_rx(packed, sizeof packed);
    };
    for(int i = 0; i < 2; ++i) { feed(sa, idle.data()); feed(sb, idle.data()); }
    double next_report = 1.0;
    while(call_time(sa) < seconds) {
        for(Side *s : sides) {
            s->board->run(100000);
            const auto &tx = s->board->dsp(0).g711_tx();
            for(size_t i = s->cursor; i < tx.size(); ++i) { s->capture.push_back(tx[i]); if(((i - s->cursor) & 1) == 0) s->mono.push_back(tx[i]); }
            s->cursor = tx.size();
        }
        // exchange complete blocks both ways
        while(sa.mono.size() - sa.consumed >= BLOCK && sb.mono.size() - sb.consumed >= BLOCK) {
            feed(sb, sa.mono.data() + sa.consumed);   // A's audio is B's input
            feed(sa, sb.mono.data() + sb.consumed);
            sa.consumed += BLOCK; sb.consumed += BLOCK;
        }
        if(std::getenv("MICA_RX") && call_time(sa) >= next_report) {
            std::fprintf(stderr, "t=%.0f  A rx words %llu ones %.3f | B rx words %llu ones %.3f\n", next_report,
                (unsigned long long)sa.words, sa.words ? sa.ones / (16.0 * sa.words) : 0.0,
                (unsigned long long)sb.words, sb.words ? sb.ones / (16.0 * sb.words) : 0.0);
            sa.words = sa.ones = sb.words = sb.ones = 0; next_report += 1.0;
        }
    }
    if(!cap_a.empty()) { std::ofstream o(cap_a, std::ios::binary); o.write((const char *)sa.capture.data(), sa.capture.size()); }
    if(!cap_b.empty()) { std::ofstream o(cap_b, std::ios::binary); o.write((const char *)sb.capture.data(), sb.capture.size()); }
    if(const char *d = std::getenv("DUMP_WORDS")) {
        for(Side *s : sides) { std::string p = std::string(d) + s->name; std::ofstream o(p, std::ios::binary); o.write((const char *)s->all.data(), s->all.size() * 2); }
    }
    std::fprintf(stderr, "A final state %s, B final state %s\n", channel_state_name(sa.state), channel_state_name(sb.state));
    return 0;
}
