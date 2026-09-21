# Reused components

## HaritoraX, SlimeTora, and the Shora TUI

`src/haritorax/` reuses Shora's Rust port of
[JovannMC/haritorax-interpreter](https://github.com/JovannMC/haritorax-interpreter)
and its [SlimeTora](https://github.com/OCSYT/SlimeTora)-compatible tracker identity
generation. The reused Shora revision is
`8e11b57b2bc2c3e752188cc71853341580377568`. `src/ui/` and file logging adapt Shora's
ratatui frontend and logging approach for the native server.

The integration covers GX6/GX2 serial acquisition, HaritoraX 2/Wireless IMU
interpretation, stable IDs, battery and button reports. It adds managed worker
lifecycle, direct registry input, shared status and commands, and a new Qt frontend.
It does not embed the complete SlimeTora application or JavaScript package.

- SlimeTora: Copyright (c) 2024 BracketProto & JovannMC,
  [MIT notice](licenses/SlimeTora-MIT.txt).
- haritorax-interpreter: Copyright (c) 2024 JovannMC,
  [MIT notice](licenses/haritorax-interpreter-MIT.txt).
- Shora: MIT OR Apache-2.0; its MIT notice matches [LICENSE-MIT](LICENSE-MIT).

The optional Qt frontend uses separately installed PySide6. Qt/PySide6 retain
their own upstream licensing; they are not relicensed by this project.

## oscavmgr face/eye integration

`src/face/` adapts the face/eye pipeline from the adjacent Shora repository,
commit `8e11b57b2bc2c3e752188cc71853341580377568`. Shora's port is based on
[galister/oscavmgr](https://github.com/galister/oscavmgr):

- Unified and combined expression model, Meta face-weight mapping.
- Babble / EyeTrackVR address mappings.
- OSC avatar parameter mapping and bundle output.
- OpenXR headless face/eye source.

The server-specific configuration, worker lifecycle, socket handling, OSCQuery
integration, fixed UniFT output, and VRChat avatar JSON input-route importer were
adapted or added here. This is an embedded subset, not the
complete oscavmgr application. Autopilot, Gogo Loco, external storage, ALVR,
and non-Meta OpenXR face providers are not included.

oscavmgr is Copyright (c) 2023 galister, MIT licensed. Its full license is retained
in [licenses/oscavmgr-MIT.txt](licenses/oscavmgr-MIT.txt). Shora is distributed under
MIT OR Apache-2.0; its MIT notice matches this project's [LICENSE-MIT](LICENSE-MIT).
