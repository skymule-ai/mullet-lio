# FAST-LIO runtime configuration

This directory is mounted read-only at `/config` in the FAST-LIO container, which loads
`/config/mid360.yaml`.

`mid360.yaml` is upstream's MID-360 configuration with the changes marked `mullet`. A drone's
values (body extrinsic, `blind`, noise) live in
[mullet-localization](https://github.com/skymule-ai/mullet-localization) (`config/fast-lio/`).
[docs/fast-lio.md](../../docs/fast-lio.md) explains each setting.
