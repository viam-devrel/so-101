#!/bin/bash

# nlopt dependency installer for SO-101 arm module
# Installs nlopt on Debian systems, provides guidance for other platforms

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() {
  echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
  echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
  echo -e "${RED}[ERROR]${NC} $1"
}

# Check if nlopt is already installed
is_nlopt_installed() {
  local has_headers=false
  local has_library=false

  # Check for development headers
  if dpkg -l | grep -q "libnlopt-dev" || [[ -f /usr/include/nlopt.h ]]; then
    has_headers=true
  fi

  # Check for runtime library
  if dpkg -l | grep -q "libnlopt0" || ldconfig -p | grep -q "libnlopt"; then
    has_library=true
  fi

  if [[ "$has_headers" == true && "$has_library" == true ]]; then
    return 0 # Already installed
  else
    return 1 # Not installed
  fi
}

# Check if we're on Linux
check_linux() {
  if [[ "$(uname -s)" != "Linux" ]]; then
    log_warn "Non-Linux system detected: $(uname -s)"
    log_warn "Please install the nlopt development library manually:"
    log_warn "  - On macOS: brew install nlopt"
    log_warn "  - On Windows: Install nlopt via vcpkg or conda"
    log_warn "  - On other systems: See https://nlopt.readthedocs.io/en/latest/NLopt_Installation/"
    exit 0
  fi
}

# Check if we're on Debian
check_debian() {
  if [[ ! -f /etc/debian_version ]]; then
    log_warn "Non-Debian Linux system detected"
    log_warn "Please install the nlopt development library using your system's package manager:"
    log_warn "  - Ubuntu/Debian: apt-get install libnlopt-dev"
    log_warn "  - CentOS/RHEL/Fedora: dnf install nlopt-devel (or yum install nlopt-devel)"
    log_warn "  - Alpine: apk add nlopt-dev"
    log_warn "  - Arch: pacman -S nlopt"
    exit 0
  fi

  if ! command -v apt-get &>/dev/null; then
    log_warn "apt-get not found on what appears to be a Debian system"
    log_warn "Please install nlopt manually or ensure apt-get is available"
    exit 0
  fi
}

# Install nlopt using apt
install_nlopt() {
  log_info "Installing nlopt development packages"

  # Update package list
  log_info "Updating package list"
  apt-get update

  # Install nlopt packages
  # libnlopt-dev: development headers and static libraries
  # libnlopt0: runtime shared library
  log_info "Installing libnlopt-dev and libnlopt0"
  apt-get install -y libnlopt-dev libnlopt0
}

# Verify installation was successful
verify_installation() {
  log_info "Verifying nlopt installation"

  # Check for headers
  if [[ ! -f /usr/include/nlopt.h ]]; then
    log_error "nlopt headers not found at /usr/include/nlopt.h"
    return 1
  fi

  return 0
}

main() {
  log_info "SO-101 arm module nlopt dependency installer"

  # Check if we're on Linux first
  check_linux

  # Check if we're on Debian
  check_debian

  # Now we know we're on Debian, check if running as root
  if [[ $EUID -ne 0 ]]; then
    log_error "This script must be run as root on Debian (use sudo)"
    exit 1
  fi

  # Check current architecture
  local arch=$(uname -m)
  log_info "Detected Debian system with architecture: $arch"

  if [[ "$arch" != "x86_64" && "$arch" != "aarch64" ]]; then
    log_warn "Unsupported architecture: $arch (expected x86_64 or aarch64)"
    log_warn "Proceeding anyway - nlopt should work on most architectures"
  fi

  # Check if nlopt is already installed
  if is_nlopt_installed; then
    log_info "nlopt is already installed on this system"
    verify_installation
    log_info "No installation needed - nlopt dependency satisfied"
    exit 0
  fi

  log_info "nlopt not found - proceeding with installation"

  # Install nlopt
  install_nlopt

  # Verify the installation worked
  if verify_installation; then
    log_info "nlopt installation completed successfully"
  else
    log_error "nlopt installation verification failed"
    exit 1
  fi
}

main "$@"
