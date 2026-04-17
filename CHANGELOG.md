# Changelog

All notable changes to `mono_edgetam` are documented in this file.

tros_1.0.0 (2026-04-17)
------------------
#### mono_edgetam_prompt
1. Supports two input modes: local image inference and subscribed image-stream inference.
2. Supports two prompt styles via `prompt_mode`: point prompts or box prompts.
3. Exports prompt initialization results (memory feature files) for downstream tracking, with optional local render output.

#### mono_edgetam_track
1. Loads memory features saved by `mono_edgetam_prompt` and uses them as tracking initialization.
2. Supports continuous tracking in both local-image and subscribed-stream modes.
3. Updates tracking memory frame by frame and publishes segmentation/tracking results.
