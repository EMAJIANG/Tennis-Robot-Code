#!/bin/bash

echo "🔑 Step 1: Checking for SSH key..."

KEY_PATH="$HOME/.ssh/id_ed25519"

if [ ! -f "$KEY_PATH" ]; then
    echo "⚙️ No SSH key found. Generating one..."
    ssh-keygen -t ed25519 -C "your_email@example.com" -f "$KEY_PATH" -N ""
else
    echo "✅ SSH key already exists at $KEY_PATH"
fi

echo ""
echo "📋 Step 2: Copy the following public key to your GitHub SSH settings:"
echo "👉 GitHub SSH key settings: https://github.com/settings/keys"
echo ""
cat "$KEY_PATH.pub"
echo ""
read -p "⏳ Press ENTER after you've added the key to GitHub..."

echo ""
echo "🔗 Step 3: Switching Git remote to SSH..."
git remote set-url origin git@github.com:EMAJIANG/Tennis-Robot-Code.git

echo ""
echo "🚀 Step 4: Pushing ROS2-Codes branch with Git LFS..."
git lfs install
git push origin ROS2-Codes

echo "✅ Done!"
