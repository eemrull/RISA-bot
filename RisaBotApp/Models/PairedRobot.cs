namespace RisaBotApp.Models;

/// <summary>
/// Local persistence model for a paired robot stored in MAUI Preferences under key "paired_robot".
/// Remembers the target Wi-Fi network SSID, mDNS hostname, and last assigned wlan0 IP.
/// </summary>
public class PairedRobot
{
    public string Serial { get; set; } = string.Empty;
    public string Name { get; set; } = string.Empty;
    public string Model { get; set; } = string.Empty;
    public string FirmwareVersion { get; set; } = string.Empty;

    /// <summary>The Wi-Fi network SSID that the robot's wlan0 was provisioned to connect to.</summary>
    public string TargetNetworkSsid { get; set; } = string.Empty;

    /// <summary>mDNS hostname used to reach this robot (e.g. "risabot10.local").</summary>
    public string MdnsHostname { get; set; } = string.Empty;

    /// <summary>The wlan0 IP address assigned at setup time (used as a direct IP fallback if mDNS fails).</summary>
    public string Wlan0Ip { get; set; } = string.Empty;

    /// <summary>Dashboard URL received on successful WiFi connect.</summary>
    public string DashboardUrl { get; set; } = string.Empty;

    public DateTime PairedAt { get; set; } = DateTime.UtcNow;
}
