#!/bin/bash

# Find all CMakeLists.txt files in the src directory
find /home/Dojo/Dojo/src -name "CMakeLists.txt" -type f | while read -r file; do
    echo "Fixing $file"
    # Replace 'action(' with 'install('
    sed -i 's/action(/install(/g' "$file"
done

echo "All CMakeLists.txt files have been updated."
