"""Check the assembled PIO TX waveform against an independent 8N1 frame."""
import re
import argparse
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--flightcode-root', type=Path, help='Optional FlightCode checkout for packet comparison')
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
header = (root / 'build-release/crsf_tx.pio.h').read_text()
program_text = header.split('crsf_tx_program_instructions[] = {', 1)[1].split('};', 1)[0]
program = [int(word, 16) for word in re.findall(r'0x([0-9a-fA-F]{4})', program_text)]
frame_text = (root / 'src/drivers/receiver/crsf_bind.h').read_text().split('crsf_bind_frame[] = {', 1)[1].split('};', 1)[0]
frame = bytes(int(word, 16) for word in re.findall(r'0x([0-9a-fA-F]{2})', frame_text))
if args.flightcode_root:
    reference_text = (args.flightcode_root / 'src/drivers/receiver/crsf_bind.h').read_text()
    reference = bytes(int(word, 16) for word in re.findall(r'0x([0-9a-fA-F]{2})', reference_text.split('crsf_bind_frame[] = {', 1)[1].split('};', 1)[0]))
    assert frame == reference, 'Pico bind bytes differ from FlightCode'
    print('FlightCode bind packet parity: PASS')

def waveform(data):
    fifo = iter(data)
    pc = x = osr = 0
    line = 1
    trace = []
    for _ in range(10000):
        instruction = program[pc]
        if instruction & 0x1000:  # optional side-set enable
            line = (instruction >> 11) & 1
        cycles = 1 + ((instruction >> 8) & 7)
        op = instruction >> 13
        next_pc = (pc + 1) % len(program)
        if op == 4 and instruction & 0x80:  # PULL
            try:
                osr = next(fifo)
            except StopIteration:
                # TXSTALL occurs as the last stop bit starts; the driver must
                # keep driving high for at least one bit before releasing TX.
                trace.extend([line] * 8)
                return trace
        elif op == 7 and (instruction >> 5) & 7 == 1:  # SET X
            x = instruction & 31
        elif op == 3 and (instruction >> 5) & 7 == 0:  # OUT PINS
            assert instruction & 31 == 1
            line, osr = osr & 1, osr >> 1
        elif op == 0 and (instruction >> 5) & 7 == 2:  # JMP X--
            if x:
                next_pc = instruction & 31
            x = (x - 1) & 0xffffffff
        else:
            raise AssertionError(f'Unsupported instruction {instruction:04x}')
        trace.extend([line] * cycles)
        pc = next_pc
    raise AssertionError('PIO did not finish')

for payload in [frame, bytes(range(256))]:
    expected = [1] * 8  # initial idle PULL
    for byte in payload:
        bits = [0] + [(byte >> bit) & 1 for bit in range(8)] + [1]
        expected.extend(level for bit in bits for level in [bit] * 8)
    assert waveform(payload) == expected, 'PIO start/data/stop timing differs from 8N1'
source = (root / 'src/drivers/receiver/sbus_receiver.c').read_text()
assert 'if (sent) sleep_us(3u)' in source, 'Final stop bit needs a hold after TXSTALL'
assert 3e-6 >= 1 / 420000, 'Stop hold is shorter than one bit'
print('Assembled PIO 8N1 waveform (bind + all 256 byte values): PASS')
print('Final stop hold: 3 us >= 2.381 us at 420000 baud; physical GPIO measurement still required.')
