import os
import soundfile as sf
from scipy.signal import resample
import numpy as np

audio_dir = "audio"
target_samplerate = 48000

for file in os.listdir(audio_dir):
    if file.endswith(".wav"):
        path = os.path.join(audio_dir, file)
        data, samplerate = sf.read(path, dtype='float32')

        if samplerate == target_samplerate:
            print(f"[OK] {file} is already {target_samplerate} Hz.")
            continue

        print(f"[FIX] {file}: {samplerate} Hz -> {target_samplerate} Hz")

        num_samples = int(len(data) * target_samplerate / samplerate)

        # Resample (handle mono or stereo)
        if data.ndim == 1:
            data_resampled = resample(data, num_samples)
        else:
            data_resampled = np.stack([
                resample(data[:, ch], num_samples) for ch in range(data.shape[1])
            ], axis=-1)

        # Overwrite or save as new file
        sf.write(path, data_resampled, target_samplerate)
        print(f"    → Resampled and saved: {file}")
