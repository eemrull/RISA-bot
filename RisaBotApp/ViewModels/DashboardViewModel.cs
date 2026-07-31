using System.Collections.ObjectModel;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using RisaBotApp.Models;
using RisaBotApp.Services;

namespace RisaBotApp.ViewModels;

/// <summary>
/// ViewModel for DashboardPage.
/// Integrates with port 8080 ROS 2 web dashboard node (dashboard.py), polling live telemetry
/// every 500ms and streaming live camera feeds.
/// </summary>
public partial class DashboardViewModel : ObservableObject, IAsyncDisposable
{
    private readonly ApiClient           _api;
    private readonly RobotPairingService _pairing;

    private CancellationTokenSource? _pollCts;
    private const int PollingIntervalMs = 500; // 500ms fast live telemetry loop

    // Network/service status changes slowly, so it rides a slower cadence than telemetry.
    private static readonly TimeSpan StatusPollInterval = TimeSpan.FromSeconds(3);
    private DateTime _lastStatusPoll = DateTime.MinValue;

    public DashboardViewModel(ApiClient api, RobotPairingService pairing)
    {
        _api     = api;
        _pairing = pairing;

        LoadPairedRobotInfo();
    }

    // ── Live ROS 2 Telemetry Properties (Port 8080 /data) ─────────────────────

    [ObservableProperty] private bool   _rosActive;
    [ObservableProperty] private bool   _autoMode;
    [ObservableProperty] private string _stateName            = "STANDBY";
    [ObservableProperty] private int    _lap                  = 1;
    [ObservableProperty] private double _speed;
    [ObservableProperty] private double _distance;
    [ObservableProperty] private double _odomX;
    [ObservableProperty] private double _odomY;
    [ObservableProperty] private double _odomYaw;
    [ObservableProperty] private double _imuRoll;
    [ObservableProperty] private double _imuPitch;
    [ObservableProperty] private double _imuYaw;
    [ObservableProperty] private double _laneError;

    // Perception badges
    [ObservableProperty] private string _trafficLight         = "NONE";
    [ObservableProperty] private bool?  _boomGate;
    [ObservableProperty] private bool?  _tunnelDetected;
    [ObservableProperty] private bool?  _parkingSignDetected;

    // Network / Portal Properties (Port 8000)
    [ObservableProperty] private string  _wlan0State          = "connected";
    [ObservableProperty] private string  _connectedSsid       = "—";
    [ObservableProperty] private bool    _internetOnline;
    [ObservableProperty] private string? _telemetryError;

    // ── Camera Tab Properties & Controls ────────────────────────────────────

    [ObservableProperty] private string _cameraStreamUrl      = string.Empty;
    [ObservableProperty] private string _activeCamView        = "raw";

    // ── Controls Tab ────────────────────────────────────────────────────────

    [ObservableProperty] private bool    _launchActive;
    [ObservableProperty] private string  _serviceState        = "active";
    [ObservableProperty] private bool    _isActionBusy;
    [ObservableProperty] private string? _actionError;
    [ObservableProperty] private ObservableCollection<LogEntry> _logs = [];
    [ObservableProperty] private string  _logsSummary         = string.Empty;

    // ── Settings Tab ────────────────────────────────────────────────────────

    [ObservableProperty] private string _robotName            = "—";
    [ObservableProperty] private string _robotSerial          = "—";
    [ObservableProperty] private string _robotModel           = "—";
    [ObservableProperty] private string _robotFirmware        = "—";
    [ObservableProperty] private string _robotMdns            = "—";
    [ObservableProperty] private string _pairedAt             = "—";
    [ObservableProperty] private string _appVersion           = AppInfo.VersionString;

    // ── Lifecycle & Polling ──────────────────────────────────────────────────

    public void StartPolling()
    {
        CameraStreamUrl = _api.CameraStreamUrl;
        _pollCts = new CancellationTokenSource();
        _ = Task.Run(() => PollLoopAsync(_pollCts.Token));
    }

    public async Task StopPollingAsync()
    {
        if (_pollCts is not null)
        {
            await _pollCts.CancelAsync();
            _pollCts.Dispose();
            _pollCts = null;
        }
    }

    private async Task PollLoopAsync(CancellationToken ct)
    {
        while (!ct.IsCancellationRequested)
        {
            await RefreshTelemetryAsync(ct);
            try { await Task.Delay(PollingIntervalMs, ct); }
            catch (TaskCanceledException) { break; }
        }
    }

    private async Task RefreshTelemetryAsync(CancellationToken ct)
    {
        // 1. Poll Port 8080 ROS 2 live data feed (/data)
        var rosRes = await _api.GetRosDashboardDataAsync(ct);
        if (rosRes.IsSuccess && rosRes.Data is not null)
        {
            var d = rosRes.Data;
            RosActive           = true;
            AutoMode            = d.AutoMode;
            StateName           = !string.IsNullOrWhiteSpace(d.State) ? d.State : "RUNNING";
            Lap                 = d.Lap;
            Speed               = d.Speed;
            Distance            = d.Distance;
            OdomX               = d.OdomX;
            OdomY               = d.OdomY;
            OdomYaw             = d.OdomYaw;
            ImuRoll             = d.ImuRoll;
            ImuPitch            = d.ImuPitch;
            ImuYaw              = d.ImuYaw;
            LaneError           = d.LaneError;
            TrafficLight        = !string.IsNullOrWhiteSpace(d.TrafficLight) ? d.TrafficLight.ToUpper() : "NONE";
            BoomGate            = d.BoomGate;
            TunnelDetected      = d.TunnelDetected;
            ParkingSignDetected = d.ParkingSignDetected;
            TelemetryError      = null;
        }
        else
        {
            RosActive = false;
        }

        // 2. Secondary poll port 8000 for network info (every ~3s)
        var now = DateTime.UtcNow;
        if (now - _lastStatusPoll >= StatusPollInterval)
        {
            _lastStatusPoll = now;

            var statusRes = await _api.GetStatusAsync(ct);
            if (statusRes.IsSuccess)
            {
                var s = statusRes.Data!;
                Wlan0State    = s.Wlan0State;
                ConnectedSsid = s.ConnectedSsid ?? "—";
                InternetOnline = s.InternetOnline;
                LaunchActive  = s.LaunchActive;
                ServiceState  = s.LaunchActive ? "active" : "inactive";
            }
        }
    }

    // ── Camera Commands ──────────────────────────────────────────────────────

    [RelayCommand]
    private async Task SelectCamViewAsync(string view)
    {
        if (string.IsNullOrWhiteSpace(view)) return;
        ActiveCamView = view;
        await _api.SetCameraViewAsync(view);
    }

    // ── Control Commands ─────────────────────────────────────────────────────

    [RelayCommand]
    private async Task ResetOdometryAsync()
    {
        IsActionBusy = true;
        ActionError  = null;
        var res = await _api.ResetOdometryAsync();
        IsActionBusy = false;
        if (!res.IsSuccess) ActionError = res.ErrorMessage;
    }

    [RelayCommand]
    private async Task SendCompetitionCommandAsync(string command)
    {
        if (string.IsNullOrWhiteSpace(command)) return;
        IsActionBusy = true;
        ActionError  = null;
        var res = await _api.SendCompetitionCommandAsync(command);
        IsActionBusy = false;
        if (!res.IsSuccess) ActionError = res.ErrorMessage;
    }

    [RelayCommand]
    private async Task StartLaunchAsync()
    {
        IsActionBusy = true;
        ActionError  = null;
        var result = await _api.StartLaunchAsync();
        IsActionBusy = false;
        if (result.IsSuccess)
        {
            LaunchActive = result.Data!.LaunchActive;
            ServiceState = result.Data.ServiceState;
        }
        else ActionError = result.ErrorMessage;
    }

    [RelayCommand]
    private async Task StopLaunchAsync()
    {
        IsActionBusy = true;
        ActionError  = null;
        var result = await _api.StopLaunchAsync();
        IsActionBusy = false;
        if (result.IsSuccess)
        {
            LaunchActive = result.Data!.LaunchActive;
            ServiceState = result.Data.ServiceState;
        }
        else ActionError = result.ErrorMessage;
    }

    [RelayCommand]
    private async Task RestartRosAsync()
    {
        IsActionBusy = true;
        ActionError  = null;
        var result = await _api.RestartRosAsync();
        IsActionBusy = false;
        if (!result.IsSuccess) ActionError = result.ErrorMessage;
    }

    [RelayCommand]
    private async Task LoadLogsAsync()
    {
        var result = await _api.GetLogsAsync();
        if (!result.IsSuccess) { ActionError = result.ErrorMessage; return; }

        var data = result.Data!;
        LogsSummary = data.StatusSummary;
        Logs.Clear();

        foreach (var line in data.Logs)
            Logs.Add(new LogEntry { Text = line, IsError = data.Errors.Contains(line) });
    }

    [RelayCommand]
    private async Task ForgetRobotAsync()
    {
        bool confirmed = await Shell.Current.DisplayAlertAsync(
            "Forget Robot",
            "This will remove the pairing and return to Discovery. Are you sure?",
            "Forget", "Cancel");

        if (!confirmed) return;

        await StopPollingAsync();
        _pairing.ClearPairedRobot();
        _api.UpdateBaseUrl("http://192.168.4.1:8000");

        await Shell.Current.GoToAsync("//DiscoveryPage");
    }

    // ── Helpers ──────────────────────────────────────────────────────────────

    private void LoadPairedRobotInfo()
    {
        var robot = _pairing.GetPairedRobot();
        if (robot is null) return;

        RobotName     = robot.Name;
        RobotSerial   = robot.Serial;
        RobotModel    = robot.Model;
        RobotFirmware = robot.FirmwareVersion;
        RobotMdns     = robot.MdnsHostname;
        PairedAt      = robot.PairedAt.ToLocalTime().ToString("yyyy-MM-dd HH:mm");
    }

    public async ValueTask DisposeAsync() => await StopPollingAsync();
}
