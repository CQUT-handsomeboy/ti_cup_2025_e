# Define the directories to search for and remove
$directoriesToDelete = @("Debug", "settings", "Release", "Output", "DebugConfig", "Listings", "Objects")

# Define the file patterns to search for and delete
$filePatternsToDelete = @("*.uvguix.*", "*.uvoptx", "*.tmp", "*.bak", "*.dep", "*.sfr", "Backup*")

# Recursively search for and remove directories with the specified names
foreach ($dir in $directoriesToDelete) {
    $items = Get-ChildItem -Path . -Include $dir -Directory -Recurse -ErrorAction SilentlyContinue
    foreach ($item in $items) {
        Remove-Item -Path $item.FullName -Recurse -Force
        Write-Host "namelesstech.ps1已删除目录 $($item.FullName)"
    }
}

# Recursively search for and remove files matching the specified patterns
foreach ($pattern in $filePatternsToDelete) {
    $items = Get-ChildItem -Path . -Include $pattern -File -Recurse -ErrorAction SilentlyContinue
    foreach ($item in $items) {
        Remove-Item -Path $item.FullName -Force
        Write-Host "namelesstech.ps1已删除文件 $($item.FullName)"
    }
}

# Pause to keep the console window open
Write-Host "按任意键继续..."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")