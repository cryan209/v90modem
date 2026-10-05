"""Which part of H.245 the DSVD terminal speaks (V.75 Tables 3-6 + Cor.1).

Shared by the schema generator and the fuzzer, so what is generated and what
is tested cannot drift apart.

Pruning never changes an encoding.  A CHOICE alternative that is dropped is
still *counted* (its index and the width of the index field are unchanged); it
just has no schema, so the encoder refuses it and the decoder reports it as
unsupported.  A dropped OPTIONAL member keeps its presence bit.  Extension
additions are always skippable on decode (they are length-prefixed open
types), whether or not they are known.

Paths: "Type" for a named type, "Type.member.member" for inline types.
"""

AUDIO = ["g711Alaw64k", "g711Alaw56k", "g711Ulaw64k", "g711Ulaw56k", "g722-64k",
         "g722-56k", "g722-48k", "g7231", "g728", "g729", "g729AnnexA",
         "g729wAnnexB", "g729AnnexAwAnnexB"]
APP = ["t120", "userData", "t434", "dsvdControl"]

CHOICE_KEEP = {
    "MultimediaSystemControlMessage": ["request", "response", "command"],
    "RequestMessage": ["terminalCapabilitySet", "openLogicalChannel",
                       "closeLogicalChannel", "requestMode"],
    "ResponseMessage": ["terminalCapabilitySetAck", "terminalCapabilitySetReject",
                        "openLogicalChannelAck", "openLogicalChannelReject",
                        "closeLogicalChannelAck", "requestModeAck",
                        "requestModeReject"],
    "CommandMessage": ["endSessionCommand"],
    "MultiplexCapability": ["v76Capability"],
    "Capability": ["receiveAudioCapability", "transmitAudioCapability",
                   "receiveAndTransmitAudioCapability",
                   "receiveDataApplicationCapability",
                   "transmitDataApplicationCapability",
                   "receiveAndTransmitDataApplicationCapability"],
    "AudioCapability": AUDIO,
    "DataApplicationCapability.application": APP,
    "DataProtocolCapability": ["v14buffered", "v42lapm", "hdlcFrameTunnelling",
                               "transparent", "segmentationAndReassembly",
                               "hdlcFrameTunnelingwSAR", "v120", "v76wCompression"],
    "DataType": ["nullData", "audioData", "data"],
    "OpenLogicalChannel.forwardLogicalChannelParameters.multiplexParameters":
        ["v76LogicalChannelParameters"],
    "OpenLogicalChannel.reverseLogicalChannelParameters.multiplexParameters":
        ["v76LogicalChannelParameters"],
    "EndSessionCommand": ["disconnect", "gstnOptions"],
    "ModeElementType": ["audioMode", "dataMode"],
    "AudioMode": [a for a in AUDIO if a != "g7231"] + ["g7231"],
    "DataMode.application": APP,
}

MEMBER_DROP = {
    "TerminalCapabilitySet.genericInformation",
    "TerminalCapabilitySetAck.genericInformation",
    "TerminalCapabilitySetReject.genericInformation",
    "OpenLogicalChannel.separateStack",
    "OpenLogicalChannel.encryptionSync",
    "OpenLogicalChannel.genericInformation",
    "OpenLogicalChannelAck.separateStack",
    "OpenLogicalChannelAck.forwardMultiplexAckParameters",
    "OpenLogicalChannelAck.encryptionSync",
    "OpenLogicalChannelAck.genericInformation",
    "OpenLogicalChannelAck.dtlsSecurityCapability",
    "OpenLogicalChannelAck.reverseLogicalChannelParameters.multiplexParameters",
    "OpenLogicalChannelReject.genericInformation",
    "ModeElement.h223ModeParameters",
    "ModeElement.h2250ModeParameters",
    "ModeElement.genericModeParameters",
    "ModeElement.multiplexedStreamModeParameters",
}

ROOT = "MultimediaSystemControlMessage"
