from __future__ import annotations

from Application.OOK_detection.ook import classify_ook


def run_ook_worker(sample_queue, result_queue, duration_s, candidates, min_confidence):
    """
    Collects (brightness, timestamp) samples from queue until sentinel or timeout.
    Runs classify_ook and puts result dict on result_queue.
    """
    samples = []
    timestamps = []

    while True:
        item = sample_queue.get()
        if item is None:
            break
        brightness, ts = item
        samples.append(brightness)
        timestamps.append(ts)

    freq, confidence = classify_ook(samples, timestamps, candidates, min_confidence)
    result_queue.put(
        {
            "freq": freq,
            "confidence": confidence,
            "samples": len(samples),
        }
    )
