using System.Collections.ObjectModel;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RisaBotApp.Models;
using RisaBotApp.Services;

namespace RisaBotApp.ViewModels;

/// <summary>
/// UI wrapper for WifiNetwork — adds emoji signal icon for DataTemplate binding.
/// </summary>
public class WifiNetworkItem
{
    public WifiNetwork Network { get; }

    public string SignalEmoji => Network.SignalBars switch
    {
        4 => "📶",
        3 => "▋▋▋",
        2 => "▋▋",
        _ => "▋"
    };

    public WifiNetworkItem(WifiNetwork network) => Network = network;
}


public partial class WifiSetupViewModel : ObservableObject, IDisposable
{
    private readonly ApiClient              _api;
    private readonly RobotConnectionService _ws;
    private readonly RobotPairingService    _pairing;

    public WifiSetupViewModel(ApiClient api, RobotConnectionService ws, RobotPairingService pairing)
    {
        _api     = api;
        _ws      = ws;
        _pairing = pairing;

        _ws.WsMessageReceived += OnWsMessage;

        // Pre-populate steps in correct protocol order
        ResetSteps();
    }

    // ── Observable properties ────────────────────────────────────────────────

    [ObservableProperty]
    private ObservableCollection<WifiNetworkItem> _networks = [];

    [ObservableProperty]
    private ObservableCollection<ConnectionStep> _steps = [];

    [ObservableProperty]
    private bool _isScanning;

    [ObservableProperty]
    private bool _isConnecting;

    [ObservableProperty]
    private string? _errorMessage;

    [ObservableProperty]
    private string? _selectedSsid;

    [ObservableProperty]
    private bool _hasError;

    // ── Commands ─────────────────────────────────────────────────────────────

    [RelayCommand]
    private async Task ScanAsync()
    {
        IsScanning   = true;
        ErrorMessage = null;
        Networks.Clear();

        var result = await _api.ScanNetworksAsync();
        IsScanning = false;

        if (result.IsSuccess)
        {
            foreach (var n in result.Data!.Networks)
                Networks.Add(new WifiNetworkItem(n));

            // Auto-select QR-scanned SSID if available
            var qrSsid = Preferences.Get("qr_ssid", string.Empty);
            if (!string.IsNullOrEmpty(qrSsid))
                SelectedSsid = qrSsid;
        }
        else
        {
            ErrorMessage = result.ErrorMessage;
        }
    }

    /// <summary>
    /// Connect to the given SSID. Called after the user enters a password.
    /// Does NOT clear the network list on failure so the user can retry.
    /// </summary>
    [RelayCommand]
    private async Task ConnectToNetworkAsync((string ssid, string password) args)
    {
        SelectedSsid = args.ssid;
        IsConnecting = true;
        HasError     = false;
        ErrorMessage = null;
        ResetSteps();

        // 1. POST /api/connect — fire-and-forget on the backend side
        var connectResult = await _api.ConnectAsync(args.ssid, args.password);
        if (!connectResult.IsSuccess)
        {
            IsConnecting = false;
            ErrorMessage = connectResult.ErrorMessage;
            HasError     = true;
            return;
        }

        // 2. Open WebSocket to /ws/log for live step progress
        await _ws.ConnectLogStreamAsync(_api.CurrentBaseUrl);
        // Receive loop fires OnWsMessage events — progress tracked there
    }

    [RelayCommand]
    private void RetryConnection()
    {
        IsConnecting = false;
        HasError     = false;
        ErrorMessage = null;
        ResetSteps();
    }

    // ── WebSocket event handler ────────────────────────────────────────────

    private async void OnWsMessage(object? sender, WsLogMessage msg)
    {
        if (msg.Type != "step") return; // ignore pong / plain log frames

        var step = Steps.FirstOrDefault(s => s.Status == msg.Status);
        if (step is not null)
        {
            // Mark previous steps as done
            foreach (var s in Steps)
            {
                if (s == step) break;
                s.State = StepState.Done;
            }

            step.State = msg.Status is "ready" or "failed"
                ? (msg.Status == "ready" ? StepState.Done : StepState.Failed)
                : StepState.Active;
        }

        if (msg.Status == "ready")
        {
            IsConnecting = false;
            await _ws.DisconnectAsync();

            // The robot is about to drop its hotspot, so the app must follow it to the
            // address it just acquired. Identity was persisted by DiscoveryViewModel
            // before it navigated here, so read it back from the pairing service.
            var paired   = _pairing.GetPairedRobot();
            var wlan0Ip  = msg.Wlan0Ip ?? string.Empty;

            // Prefer the mDNS name (survives DHCP lease changes); fall back to the IP.
            var host = paired?.MdnsHostname;
            if (string.IsNullOrWhiteSpace(host)) host = wlan0Ip;

            if (!string.IsNullOrWhiteSpace(host))
            {
                var dashboardUrl = msg.DashboardUrl ?? $"http://{host}:8080";
                _pairing.UpdateAfterProvisioning(SelectedSsid ?? string.Empty, wlan0Ip, dashboardUrl);

                // Re-point every call. ApiClient derives :8080 and :1984 from this base,
                // so telemetry and the camera follow automatically.
                _api.UpdateBaseUrl($"http://{host}:8000");
            }

            // Clear QR Preferences now that pairing is complete
            Preferences.Remove("qr_ssid");
            Preferences.Remove("qr_password");

            await Shell.Current.GoToAsync("//DashboardPage");
        }
        else if (msg.Status == "failed")
        {
            IsConnecting = false;
            HasError     = true;
            ErrorMessage = msg.Message;
            await _ws.DisconnectAsync();
        }
    }

    // ── Helpers ────────────────────────────────────────────────────────────

    private void ResetSteps()
    {
        Steps = new ObservableCollection<ConnectionStep>(
            ConnectionSteps.Ordered
                .Select(s => new ConnectionStep
                {
                    Status  = s,
                    Label   = ConnectionSteps.ToDisplayLabel(s),
                    State   = StepState.Pending
                })
        );
    }

    public void Dispose() => _ws.WsMessageReceived -= OnWsMessage;
}

/// <summary>One step in the WiFi connection progress list.</summary>
public partial class ConnectionStep : ObservableObject
{
    public string Status { get; init; } = string.Empty;
    public string Label  { get; init; } = string.Empty;

    [ObservableProperty]
    private StepState _state = StepState.Pending;

    public bool IsPending  => State == StepState.Pending;
    public bool IsActive   => State == StepState.Active;
    public bool IsDone     => State == StepState.Done;
    public bool IsFailed   => State == StepState.Failed;
}

public enum StepState { Pending, Active, Done, Failed }
