// Offline AVAudioEngine integration check: no sound or recordings.
import AVFoundation
let engine = AVAudioEngine(), player = AVAudioPlayerNode()
let format = AVAudioFormat(standardFormatWithSampleRate:8000,channels:1)!
engine.attach(player); engine.connect(player,to:engine.mainMixerNode,format:format)
try engine.enableManualRenderingMode(.offline,format:format,maximumFrameCount:800)
func append(_ count:Int) {
    let buffer = AVAudioPCMBuffer(pcmFormat:format,frameCapacity:AVAudioFrameCount(count))!
    buffer.frameLength = AVAudioFrameCount(count)
    for i in 0..<count { buffer.floatChannelData![0][i] = 0.25 }
    player.scheduleBuffer(buffer)
}
append(3200); append(2400)
try engine.start(); player.play()
let out = AVAudioPCMBuffer(pcmFormat:format,frameCapacity:800)!
for _ in 0..<3 {
    let status = try engine.renderOffline(800,to:out)
    precondition(status == .success)
    precondition((0..<800).allSatisfy { abs(out.floatChannelData![0][$0]-0.25)<0.0001 })
}
let played = player.playerTime(forNodeTime:player.lastRenderTime!)!
precondition(played.sampleTime == 2400)
precondition(5600-Int(played.sampleTime) == 3200)
append(1600)
for _ in 0..<6 {
    let status = try engine.renderOffline(800,to:out)
    precondition(status == .success)
    precondition((0..<800).allSatisfy { abs(out.floatChannelData![0][$0]-0.25)<0.0001 })
}
player.stop(); append(2400); player.play()
let finalStatus = try engine.renderOffline(800,to:out)
precondition(finalStatus == .success)
precondition(player.playerTime(forNodeTime:player.lastRenderTime!)!.sampleTime == 800)
print("Audio clock: exact partial-buffer occupancy, continuous PCM, reset origin OK")
