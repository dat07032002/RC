#!/bin/bash
# SSH Key Setup Script for School Server
# Run this ONCE on your desktop to set up passwordless SSH access

set -e

echo "🔑 SSH Key Setup for School Server"
echo "=================================="
echo ""

# Step 1: Get server details
read -p "Enter your email (for SSH key comment): " USER_EMAIL
read -p "Enter server IP address: " SERVER_IP
read -p "Enter server username: " SERVER_USER
read -p "Enter server SSH port (default 22): " SERVER_PORT
SERVER_PORT=${SERVER_PORT:-22}

# Step 2: Generate SSH key
SSH_KEY_PATH="$HOME/.ssh/server_key"
if [ -f "$SSH_KEY_PATH" ]; then
    echo "⚠️  SSH key already exists at $SSH_KEY_PATH"
    read -p "Overwrite? (y/n): " OVERWRITE
    if [ "$OVERWRITE" != "y" ]; then
        echo "Using existing key..."
        SSH_KEY_PATH="$SSH_KEY_PATH"
    fi
else
    echo "📝 Generating SSH key pair..."
    mkdir -p "$HOME/.ssh"
    chmod 700 "$HOME/.ssh"

    ssh-keygen -t ed25519 -C "$USER_EMAIL" -f "$SSH_KEY_PATH" -N ""
    chmod 600 "$SSH_KEY_PATH"
    chmod 644 "$SSH_KEY_PATH.pub"
    echo "✓ SSH key generated at $SSH_KEY_PATH"
fi

# Step 3: Display public key
echo ""
echo "📋 Your Public Key (copy this):"
echo "================================"
cat "$SSH_KEY_PATH.pub"
echo ""

# Step 4: Add public key to server
echo "🔌 Adding public key to server..."
echo ""
echo "You'll need to enter your server password ONE TIME."
echo "After this, you'll have passwordless access."
echo ""

read -p "Press Enter when ready to connect to server..."

# Create .ssh directory on server and add public key
ssh -p "$SERVER_PORT" "$SERVER_USER@$SERVER_IP" << 'SSH_SETUP'
mkdir -p ~/.ssh
chmod 700 ~/.ssh
cat >> ~/.ssh/authorized_keys
chmod 600 ~/.ssh/authorized_keys
echo "✓ Public key added to server"
SSH_SETUP < "$SSH_KEY_PATH.pub"

# Step 5: Test passwordless login
echo ""
echo "🧪 Testing passwordless login..."
if ssh -i "$SSH_KEY_PATH" -p "$SERVER_PORT" "$SERVER_USER@$SERVER_IP" "echo ✓ Connection successful!" 2>/dev/null; then
    echo "✓ Passwordless SSH working!"
else
    echo "⚠️  Connection test failed. Check IP and credentials."
fi

# Step 6: Create SSH config entry
echo ""
echo "⚙️  Creating SSH config entry..."
SSH_CONFIG="$HOME/.ssh/config"
if grep -q "Host school-server" "$SSH_CONFIG" 2>/dev/null; then
    echo "⚠️  school-server entry already in SSH config"
else
    cat >> "$SSH_CONFIG" << EOF

Host school-server
    HostName $SERVER_IP
    User $SERVER_USER
    IdentityFile $SSH_KEY_PATH
    Port $SERVER_PORT
EOF
    chmod 600 "$SSH_CONFIG"
    echo "✓ Added 'school-server' to SSH config"
fi

echo ""
echo "✅ Setup Complete!"
echo "================================"
echo ""
echo "📌 You can now use:"
echo "   ssh school-server"
echo ""
echo "📌 Instead of:"
echo "   ssh -i $SSH_KEY_PATH -p $SERVER_PORT $SERVER_USER@$SERVER_IP"
echo ""
echo "🚀 Test it with:"
echo "   ssh school-server 'echo Hello from server!'"
echo ""
