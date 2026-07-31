using System.Collections.ObjectModel;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RisaBotApp.Models;
using RisaBotApp.Services;

namespace RisaBotApp.ViewModels;

/// <summary>
/// ViewModel for DiscoveryPage implementing NoMachine-style choice-based device selection grid.
/// </summary>
public partial class DiscoveryViewModel : ObservableObject
{
    private readonly ApiClient _api;
    private readonly RobotPairingService _pairing;
    private readonly RobotDiscoveryService _discovery;

    public DiscoveryViewModel(ApiClient api, RobotPairingService pairing, RobotDiscoveryService discovery)
    {
        _api       = api;
        _pairing   = pairing;
        _discovery = discovery;
    }

    // ── Observable properties ───────────────────────────────────────────────

    [ObservableProperty]
    private bool _isScanning;

    [ObservableProperty]
    [NotifyPropertyChangedFor(nameof(HasError))]
    private string? _errorMessage;

    [ObservableProperty]
    private string _statusHint = "Scanning local network and hotspot for available RISA-Bots…";

    [ObservableProperty]
    private string _manualHostInput = string.Empty;

    [ObservableProperty]
    private bool _showManualEntry;

    public ObservableCollection<DiscoveredRobotDevice> DiscoveredRobots { get; } = new();

    public bool HasRobots => DiscoveredRobots.Count > 0;
    public bool HasError  => !string.IsNullOrEmpty(ErrorMessage);

    // ── Lifecycle & Scanning ────────────────────────────────────────────────

    /// <summary>
    /// Executes parallel network discovery scan to find all online RISA-Bots.
    /// Populates the NoMachine-style choice collection.
    /// </summary>
    public async Task RunNetworkScanAsync()
    {
        if (IsScanning) return;

        IsScanning      = true;
        ErrorMessage    = null;
        StatusHint      = "Scanning network for RISA-Bots…";
        DiscoveredRobots.Clear();
        OnPropertyChanged(nameof(HasRobots));

        try
        {
            var paired = _pairing.GetPairedRobot();
            var devices = await _discovery.ScanAllRobotsAsync(paired);

            IsScanning = false;

            foreach (var dev in devices)
            {
                DiscoveredRobots.Add(dev);
            }
            OnPropertyChanged(nameof(HasRobots));

            if (DiscoveredRobots.Count > 0)
            {
                StatusHint = $"Found {DiscoveredRobots.Count} RISA-Bot device(s) on your network. Tap to connect.";
            }
            else
            {
                StatusHint      = "No RISA-Bots discovered automatically. Make sure your robot is powered on and connected to the same Wi-Fi network, or enter host manually.";
                ShowManualEntry = true;
            }
        }
        catch (Exception ex)
        {
            IsScanning      = false;
            ErrorMessage    = $"Discovery error: {ex.Message}";
            ShowManualEntry = true;
        }
    }

    // ── Device Selection ────────────────────────────────────────────────────

    /// <summary>
    /// Tapped by user on a robot choice card in the NoMachine-style grid.
    /// Connects to the target robot, remembers its mDNS/IP in background, and navigates.
    /// </summary>
    [RelayCommand]
    private async Task SelectRobotAsync(DiscoveredRobotDevice device)
    {
        if (device is null) return;

        try
        {
            // Set base URL for ApiClient
            _api.UpdateBaseUrl(device.BaseUrl);

            // Remember/Pair this robot
            var hostOrIp = device.Info.MdnsHostname ?? device.Info.Hostname;
            _pairing.SaveManualPairing(device.Info, hostOrIp);

            if (device.IsHotspot)
            {
                // Unconfigured device on hotspot → navigate to Wi-Fi Setup page
                await Shell.Current.GoToAsync("//WifiSetupPage");
            }
            else
            {
                // Configured device on local Wi-Fi → navigate straight to Dashboard
                await Shell.Current.GoToAsync("//DashboardPage");
            }
        }
        catch (Exception ex)
        {
            ErrorMessage = $"Failed to connect to {device.DisplayName}: {ex.Message}";
        }
    }

    // ── Commands ───────────────────────────────────────────────────────────

    [RelayCommand]
    private async Task RescanNetworkAsync()
    {
        await RunNetworkScanAsync();
    }

    [RelayCommand]
    private void ToggleManualEntry()
    {
        ShowManualEntry = !ShowManualEntry;
    }

    [RelayCommand]
    private async Task ManualConnectAsync()
    {
        if (string.IsNullOrWhiteSpace(ManualHostInput))
        {
            ErrorMessage = "Please enter a valid hostname or IP address.";
            return;
        }

        IsScanning   = true;
        ErrorMessage = null;
        StatusHint   = $"Connecting to '{ManualHostInput}'…";

        try
        {
            var paired = _pairing.GetPairedRobot();
            var device = await _discovery.TryManualConnectAsync(ManualHostInput, paired);
            IsScanning = false;

            if (device is not null)
            {
                await SelectRobotAsync(device);
            }
            else
            {
                ErrorMessage = $"Could not reach robot at '{ManualHostInput}'. Ensure it is powered on and reachable.";
            }
        }
        catch (Exception ex)
        {
            IsScanning   = false;
            ErrorMessage = $"Connection error: {ex.Message}";
        }
    }
}
