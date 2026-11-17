# Backend laptop: let the robots reach MQTT, the Host, the Evidence Recorder and NTP on any network (run ONCE as admin).
#   powershell -ExecutionPolicy Bypass -File tools\laptop_lab_network.ps1
# Re-run after joining a new Wi-Fi: the new network must be "Private" for mDNS (LAPTOP-xxx.local) and inbound links.
#Requires -RunAsAdministrator
$ErrorActionPreference = 'Stop'

# 1. Wi-Fi networks -> Private (a "Public" profile drops mDNS and inbound connections). The ICS Ethernet link is
#    "Unidentified network" and stays Public; the rules below cover it explicitly, limited to the local subnet.
Get-NetConnectionProfile | Where-Object { $_.InterfaceAlias -like 'Wi-Fi*' -and $_.NetworkCategory -ne 'Private' } |
    ForEach-Object { Set-NetConnectionProfile -InterfaceIndex $_.InterfaceIndex -NetworkCategory Private; "Wi-Fi '$($_.Name)' -> Private" }

# 2. Inbound rules (robots only come from the local subnet).
$rules = @(
    @{ Name = 'WaroTrans MQTT'; Protocol = 'TCP'; Port = @(1883, 9001) },
    @{ Name = 'WaroTrans Host'; Protocol = 'TCP'; Port = @(5090) },
    @{ Name = 'WaroTrans Evidence Recorder'; Protocol = 'TCP'; Port = @(5095) },
    @{ Name = 'WaroTrans NTP'; Protocol = 'UDP'; Port = @(123) }
)
foreach ($r in $rules) {
    Get-NetFirewallRule -DisplayName $r.Name -ErrorAction SilentlyContinue | Remove-NetFirewallRule
    New-NetFirewallRule -DisplayName $r.Name -Direction Inbound -Action Allow -Protocol $r.Protocol -LocalPort $r.Port `
        -Profile Private, Public -RemoteAddress LocalSubnet | Out-Null
    "firewall: $($r.Name) $($r.Protocol) $($r.Port -join ',')"
}

# 3. Docker runs in WSL2 with networkingMode=mirrored: inbound traffic to its ports also passes the Hyper-V firewall.
$wsl = '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'
if (Get-Command New-NetFirewallHyperVRule -ErrorAction SilentlyContinue) {
    Get-NetFirewallHyperVRule -Name 'WaroTrans-MQTT' -ErrorAction SilentlyContinue | Remove-NetFirewallHyperVRule
    New-NetFirewallHyperVRule -Name 'WaroTrans-MQTT' -DisplayName 'WaroTrans MQTT (WSL)' -Direction Inbound `
        -VMCreatorId $wsl -Protocol TCP -LocalPorts 1883, 9001 -Action Allow | Out-Null
    'hyper-v firewall: WSL inbound 1883, 9001 allowed'
}

# 4. NTP server for the robots' chrony (Windows Time).
Set-ItemProperty HKLM:\SYSTEM\CurrentControlSet\Services\W32Time\TimeProviders\NtpServer -Name Enabled -Value 1
Set-ItemProperty HKLM:\SYSTEM\CurrentControlSet\Services\W32Time\Config -Name AnnounceFlags -Value 5
Set-Service w32time -StartupType Automatic
Restart-Service w32time
w32tm /resync /nowait | Out-Null
'ntp: Windows Time serves NTP'

# 5. What the robots will see.
"laptop name: $env:COMPUTERNAME.local"
Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.IPAddress -notlike '127.*' -and $_.IPAddress -notlike '169.254.*' } |
    ForEach-Object { "  $($_.InterfaceAlias): $($_.IPAddress)" }
'Start the Host and the Recorder on all interfaces: --urls http://0.0.0.0:5090 and http://0.0.0.0:5095'
