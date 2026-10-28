Synthetic checkerboard fax from MT9234ZBA-USB, wired-11, 28 October 2026.

repeated-dcs.bits contains the first 86,000 incoming descrambled bits, packed
least-significant bit first. It includes repeated TSI/DCS, primary image
frames (some CRC failures), and all three CRC-valid RCP frames. No private
document is included. The HDX regression clocks TX only on control: the old
CFR transition wait switched to primary before a repeated CFR completed,
leaving T.30 rejecting valid FCD/RCP in F_CFR.
