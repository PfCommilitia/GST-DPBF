# Save the original directory
$originalDir = Get-Location

# Change to the build directory
Set-Location -Path "build"

# Run the DPBF executable
& "bin/Debug/DPBF.exe"

# Return to the original directory
Set-Location -Path $originalDir 