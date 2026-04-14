"""Reference V.32bis building blocks."""

from .coding import (
    SUPPORTED_BIT_RATES,
    bits_per_symbol,
    encode_symbol_bits,
    differential_encode,
    convolution_encode,
)
from .constellation import symbol_to_point
from .rate_signal import (
    RATE_14400,
    RATE_12000,
    RATE_9600,
    RATE_7200,
    RATE_4800,
    SUPPORTED_RATE_MASK,
    e_sequence_bits,
    encode_rate_sequence_bits,
    list_from_rate_mask,
    rate_mask_from_list,
    rate_signal_bits,
)
from .scrambler import Descrambler, Scrambler, scrambler_tap
from .negotiation import (
    decode_e_rate,
    decode_rate_mask,
    detect_repeated_rate_signal,
    detect_s_sequence,
    highest_common_rate,
    negotiate_renegotiation_rate,
    negotiate_startup_rate,
    validate_e_sequence_bits,
    validate_rate_signal_bits,
)
from .receiver import DetectedEvent, V32bisLogicalReceiver
from .startup import generate_answer_startup_trace, generate_call_startup_trace
from .simulator import simulate_startup
from .training import (
    STATE_A,
    STATE_B,
    STATE_C,
    STATE_D,
    generate_conditioning_signal,
    generate_s_bar_segment,
    generate_s_segment,
    generate_trn_segment,
)

__all__ = [
    "SUPPORTED_BIT_RATES",
    "bits_per_symbol",
    "encode_symbol_bits",
    "differential_encode",
    "convolution_encode",
    "symbol_to_point",
    "RATE_14400",
    "RATE_12000",
    "RATE_9600",
    "RATE_7200",
    "RATE_4800",
    "SUPPORTED_RATE_MASK",
    "rate_mask_from_list",
    "list_from_rate_mask",
    "rate_signal_bits",
    "e_sequence_bits",
    "encode_rate_sequence_bits",
    "detect_s_sequence",
    "validate_rate_signal_bits",
    "validate_e_sequence_bits",
    "decode_rate_mask",
    "decode_e_rate",
    "detect_repeated_rate_signal",
    "highest_common_rate",
    "negotiate_startup_rate",
    "negotiate_renegotiation_rate",
    "DetectedEvent",
    "V32bisLogicalReceiver",
    "Scrambler",
    "Descrambler",
    "scrambler_tap",
    "STATE_A",
    "STATE_B",
    "STATE_C",
    "STATE_D",
    "generate_s_segment",
    "generate_s_bar_segment",
    "generate_trn_segment",
    "generate_conditioning_signal",
    "generate_call_startup_trace",
    "generate_answer_startup_trace",
    "simulate_startup",
]
