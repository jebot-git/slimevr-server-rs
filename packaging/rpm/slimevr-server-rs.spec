%global debug_package %{nil}
# Rust dependencies are vendored in Source0; no network is used by rpmbuild.

Name:           slimevr-server-rs
Version:        0.1.0
Release:        1%{?dist}
Summary:        Native Rust full-body tracking server and persistent socket proxy
License:        MIT AND Apache-2.0 AND BSD-3-Clause AND MPL-2.0 AND Unicode-3.0 AND Zlib
URL:            https://github.com/jebot-git/slimevr-server-rs
Source0:        %{name}-%{version}.tar.gz

BuildRequires:  cargo
BuildRequires:  rust >= 1.88
BuildRequires:  gcc
BuildRequires:  pkgconfig(libudev)
BuildRequires:  pkgconfig(openxr)
BuildRequires:  python3
BuildRequires:  python3-pyside6
BuildRequires:  desktop-file-utils
BuildRequires:  systemd-rpm-macros
Requires:       python3
%{?systemd_requires}

%description
Experimental SlimeVR-compatible tracking with native HaritoraX acquisition,
calibration, filtering, SolarXR output and UniFT/VRChat JSON face translation.
Includes a persistent socket proxy and an independent backend user service.

%package qt
Summary:        Qt6 skeleton preview and calibration frontend for Shora
BuildArch:      noarch
Requires:       %{name} = %{version}-%{release}
Requires:       python3-pyside6 >= 6.6

%description qt
PySide6 frontend with live skeleton preview, device status, calibration,
smoothing, prediction and drift controls. The desktop launcher attaches to a
running server.

%package wivrn
Summary:        WiVRn user-service integration for Shora tracking
BuildArch:      noarch
Requires:       %{name} = %{version}-%{release}
Requires:       wivrn

%description wivrn
Starts Shora and its persistent proxy with WiVRn. The proxy follows WiVRn's
lifetime. The backend can restart independently without disconnecting WiVRn.

%prep
%autosetup

%build
export CARGO_HOME="$PWD/.cargo-home"
export CARGO_TARGET_DIR="$PWD/target"
cargo build --frozen --release --features face-xr --bins -j %{_smp_build_ncpus}

%install
install -Dpm755 target/release/slimevr-server-rs %{buildroot}%{_bindir}/slimevr-server-rs
install -Dpm755 target/release/shora-proxy %{buildroot}%{_bindir}/shora-proxy
strip --strip-unneeded %{buildroot}%{_bindir}/slimevr-server-rs %{buildroot}%{_bindir}/shora-proxy
install -Dpm755 packaging/rpm/shora-qt %{buildroot}%{_bindir}/shora-qt
install -d %{buildroot}%{_datadir}/%{name}/frontend
install -pm644 frontend/*.py %{buildroot}%{_datadir}/%{name}/frontend/
chmod 755 %{buildroot}%{_datadir}/%{name}/frontend/shora_qt.py
sed -i '1c#!/usr/bin/python3' %{buildroot}%{_datadir}/%{name}/frontend/shora_qt.py
install -Dpm644 config.example.toml %{buildroot}%{_datadir}/%{name}/config.example.toml
install -Dpm644 packaging/rpm/service-default.toml %{buildroot}%{_datadir}/%{name}/service-default.toml
install -Dpm755 packaging/rpm/init-profile %{buildroot}%{_libexecdir}/%{name}/init-profile
install -Dpm755 examples/systemd/wait-sockets.py %{buildroot}%{_libexecdir}/%{name}/wait-sockets.py
install -Dpm644 packaging/rpm/shora-server.service %{buildroot}%{_userunitdir}/shora-server.service
install -Dpm644 packaging/rpm/shora-proxy.service %{buildroot}%{_userunitdir}/shora-proxy.service
install -Dpm644 examples/systemd/wivrn.service.d/shora.conf %{buildroot}%{_userunitdir}/wivrn.service.d/shora.conf
desktop-file-install --dir=%{buildroot}%{_datadir}/applications packaging/rpm/shora-qt.desktop
cp packaging/rpm/README.md RPM.md

%check
export CARGO_HOME="$PWD/.cargo-home"
export CARGO_TARGET_DIR="$PWD/target"
cargo test --frozen --release --workspace --features face-xr -j %{_smp_build_ncpus}
QT_QPA_PLATFORM=offscreen python3 tests/hub_smoke.py --server "$PWD/target/release/slimevr-server-rs" --qt
desktop-file-validate %{buildroot}%{_datadir}/applications/shora-qt.desktop

%post
%systemd_user_post shora-server.service

%preun
%systemd_user_preun shora-server.service

%post wivrn
%systemd_user_post shora-proxy.service

%preun wivrn
%systemd_user_preun shora-proxy.service

%files
%license LICENSE-MIT LICENSE-APACHE licenses/ packaging/rpm/dependency-licenses/
%doc README.md ROADMAP.md THIRD_PARTY.md RPM.md
%{_bindir}/slimevr-server-rs
%{_bindir}/shora-proxy
%dir %{_datadir}/%{name}
%{_datadir}/%{name}/config.example.toml
%{_datadir}/%{name}/service-default.toml
%{_libexecdir}/%{name}/
%{_userunitdir}/shora-server.service

%files qt
%doc RPM.md
%{_bindir}/shora-qt
%{_datadir}/%{name}/frontend/
%{_datadir}/applications/shora-qt.desktop

%files wivrn
%doc RPM.md
%{_userunitdir}/shora-proxy.service
%dir %{_userunitdir}/wivrn.service.d
%{_userunitdir}/wivrn.service.d/shora.conf

%changelog
* Mon Sep 21 2026 jebot-git <326713999+jebot-git@users.noreply.github.com> - 0.1.0-1
- Package native tracking, Qt6 frontend, face translation and WiVRn reconnect proxy.
