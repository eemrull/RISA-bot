using RisaBotApp.ViewModels;

namespace RisaBotApp.Pages;

public partial class DashboardPage : ContentPage
{
    private readonly DashboardViewModel _vm;
    private string _activeTab = "Telemetry";

    public DashboardPage(DashboardViewModel vm)
    {
        InitializeComponent();
        _vm = vm;
        BindingContext = vm;
    }

    protected override void OnAppearing()
    {
        base.OnAppearing();
        _vm.StartPolling();
        _vm.LoadLogsCommand.ExecuteAsync(null);
    }

    protected override async void OnDisappearing()
    {
        base.OnDisappearing();
        await _vm.StopPollingAsync();
    }

    // ── Tab switching ────────────────────────────────────────────────────────

    private void OnTabTapped(object? sender, EventArgs e)
    {
        if (e is not TappedEventArgs te) return;
        var tab = te.Parameter as string ?? "Telemetry";
        SwitchTab(tab);
    }

    private void SwitchTab(string tab)
    {
        _activeTab = tab;

        // Content panels
        TelemetryTab.IsVisible = tab == "Telemetry";
        CameraTab.IsVisible    = tab == "Camera";
        ControlsTab.IsVisible  = tab == "Controls";
        SettingsTab.IsVisible  = tab == "Settings";

        // Tab bar highlight
        SetTabActive(TabTelemetry, tab == "Telemetry");
        SetTabActive(TabCamera,    tab == "Camera");
        SetTabActive(TabControls,  tab == "Controls");
        SetTabActive(TabSettings,  tab == "Settings");

        // Load logs when Controls tab becomes active
        if (tab == "Controls")
            _vm.LoadLogsCommand.ExecuteAsync(null);
    }

    private static void SetTabActive(View tabItem, bool active)
    {
        tabItem.Opacity = active ? 1.0 : 0.5;
    }
}
