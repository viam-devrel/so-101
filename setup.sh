#!/bin/bash

set -e

# Check prerequisites
if ! command -v curl &> /dev/null; then
    echo "❌ curl is required but not installed. Please install curl and try again."
    exit 1
fi

if ! command -v make &> /dev/null; then
    echo "❌ make is required but not installed. Please install make and try again."
    exit 1
fi

# Install mise
echo "📦 Installing mise..."
if ! command -v mise &> /dev/null; then
    curl https://mise.run | sh
    
    # Add mise to PATH for this session
    export PATH="$HOME/.local/bin:$PATH"
    
    # Verify installation
    if ! command -v mise &> /dev/null; then
        echo "❌ mise installation failed. Please check your internet connection and try again."
        exit 1
    fi
    
    echo "✅ mise installed successfully"
else
    echo "✅ mise already installed"
fi

# Setup shell integration
echo "🔧 Setting up mise shell integration..."
eval "$(mise activate bash)"

# Install Node.js v22
echo "📦 Installing Node.js v22..."
mise use -g node@22

# Verify Node.js installation
if ! command -v node &> /dev/null; then
    echo "❌ Node.js installation failed"
    exit 1
fi

echo "✅ Node.js $(node --version) installed"

# Install pnpm
echo "📦 Installing pnpm..."
mise use -g pnpm

eval "$(mise activate bash)"

# Verify pnpm installation
if ! command -v pnpm &> /dev/null; then
    echo "❌ pnpm installation failed"
    exit 1
fi

echo "✅ pnpm $(pnpm --version) installed"

# Run project setup
echo "🛠️  Running project setup..."
make setup

echo ""
echo "🎉 Setup complete!"
