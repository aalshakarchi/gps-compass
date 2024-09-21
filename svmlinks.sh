#!/bin/bash

# Define the list of libraries you want to create symbolic links for
libraries=("libzip" "libftdi" "libusb" "libtool" "glibmm" "boost" "qt" "glib" "libdbus-1")

# Set Homebrew path based on your architecture (Intel or Apple Silicon)
brew_prefix=$(brew --prefix)

# Set the target location for symbolic links
target_dir="/usr/local/opt"  # Modify this if you need a different location

# Loop through each library and create symbolic links
for lib in "${libraries[@]}"; do
  src_dir="$brew_prefix/Cellar/$lib"
  if [ -d "$src_dir" ]; then
    # Get the latest version of the library
    latest_version=$(ls "$src_dir" | sort -V | tail -n 1)
    if [ -n "$latest_version" ]; then
      lib_src="$src_dir/$latest_version/lib"
      if [ -d "$lib_src" ]; then
        echo "Creating symbolic link for $lib..."
        sudo ln -sf "$lib_src" "$target_dir/$lib"
      else
        echo "Library directory not found for $lib"
      fi
    else
      echo "No version found for $lib"
    fi
  else
    echo "$lib not installed"
  fi
done

echo "Symbolic links creation complete."