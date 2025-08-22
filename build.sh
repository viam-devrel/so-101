#!/bin/bash

set -e

# Add mise to PATH for this session
export PATH="$HOME/.local/bin:$PATH"

echo "🔧 Setting up mise shell integration..."
eval "$(mise activate bash)"

make module.tar.gz
