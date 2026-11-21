"""Train the exercise classifier and export it as model.h for the Arduino.

Usage: python train.py [data_dir] [output_header]

Reads every labeled CSV from record.py, cuts each recording into 3 s windows,
summarizes each window into features, trains a small random forest, prints
accuracy on held-out data, then writes the model as plain C++ (via micromlgen).
"""
import sys
from pathlib import Path

import numpy as np
from micromlgen import port
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import classification_report, confusion_matrix

# 30 samples at 10 Hz = 3 s window, a new window every 10 samples (1 s),
# 8 sensor channels per sample. These are written into model.h for the sketch.
WINDOW, HOP, CHANNELS = 30, 10, 8
HERE = Path(__file__).parent


def features(window):
    """Per channel: mean, std, min, max, mean absolute step. Must match classifyForm() in the sketch."""
    return np.concatenate([[c.mean(), c.std(), c.min(), c.max(), np.abs(np.diff(c)).mean()] for c in window.T])


def windows(data):
    """Split a recording into overlapping windows of WINDOW rows, HOP rows apart."""
    return [data[s:s + WINDOW] for s in range(0, len(data) - WINDOW + 1, HOP)]


def load(data_dir):
    """Load all CSVs and return (train, test) lists of (features, label).

    The label comes from the filename, e.g. squat_good_20261120-151920.csv.
    The first 75% of each recording is for training and the rest for testing,
    skipping the windows that overlap the split so test data is unseen.
    """
    files = sorted(data_dir.glob("*.csv"))
    if not files:
        sys.exit(f"no CSVs in {data_dir}, run record.py first")
    train, test = [], []
    for f in files:
        label = f.stem.rsplit("_", 1)[0]
        data = np.loadtxt(f, delimiter=",", skiprows=1, ndmin=2)
        if data.shape[1] != CHANNELS:
            sys.exit(f"{f.name}: expected {CHANNELS} columns, got {data.shape[1]}")
        feats = [features(w) for w in windows(data)]
        cut = int(len(feats) * 0.75)
        train += [(x, label) for x in feats[:cut]]
        test += [(x, label) for x in feats[cut + WINDOW // HOP - 1:]]
    return train, test


def self_test():
    """Sanity checks on the feature and window code, run before every training."""
    flat = np.full((WINDOW, CHANNELS), 2.0)
    assert np.allclose(features(flat)[:5], [2, 0, 2, 2, 0])
    ramp = np.tile(np.arange(WINDOW, dtype=float)[:, None], CHANNELS)
    assert len(features(ramp)) == CHANNELS * 5
    assert np.isclose(features(ramp)[4], 1.0)
    assert len(windows(ramp)) == 1
    assert len(windows(np.vstack([ramp, ramp]))) == WINDOW // HOP + 1


def main():
    self_test()
    data_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "data"
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else HERE.parent / "model.h"

    train, test = load(data_dir)
    # Labels become class numbers 0..N-1; the names are stored in model.h
    classes = sorted({label for _, label in train + test})
    index = {c: i for i, c in enumerate(classes)}

    def split(rows):
        return np.array([x for x, _ in rows]), np.array([index[l] for _, l in rows])

    # 10 shallow trees keeps the exported model small enough for the Uno R4
    clf = RandomForestClassifier(n_estimators=10, max_depth=8, random_state=0)

    # Measure accuracy on the held-out part of each recording
    if test:
        clf.fit(*split(train))
        X_te, y_te = split(test)
        pred = clf.predict(X_te)
        labels = list(range(len(classes)))
        print(classification_report(y_te, pred, labels=labels, target_names=classes, zero_division=0))
        print("confusion matrix (rows = true):", classes)
        print(confusion_matrix(y_te, pred, labels=labels))

    # Final model uses all the data, then is converted to C++ if/else code
    clf.fit(*split(train + test))
    header = f"#define MODEL_WINDOW {WINDOW}\n#define MODEL_HOP {HOP}\n#define MODEL_CHANNELS {CHANNELS}\n"
    out.write_text(header + port(clf, classmap=dict(enumerate(classes))))
    print(f"wrote {out} ({len(classes)} classes, {len(train) + len(test)} windows)")


if __name__ == "__main__":
    main()
