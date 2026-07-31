using RisaBotApp.Models;
using RisaBotApp.ViewModels;

namespace RisaBotApp.Pages;

public partial class WifiSetupPage : ContentPage
{
    private readonly WifiSetupViewModel _vm;

    public WifiSetupPage(WifiSetupViewModel vm)
    {
        InitializeComponent();
        _vm = vm;
        BindingContext = vm;
    }

    protected override async void OnAppearing()
    {
        base.OnAppearing();
        // Auto-scan when page first appears
        await _vm.ScanCommand.ExecuteAsync(null);
    }

    protected override void OnDisappearing()
    {
        base.OnDisappearing();
        _vm.Dispose();
    }

    /// <summary>
    /// Network row tapped — show password prompt or connect directly if Open.
    /// Uses DisplayPromptAsync (built-in MAUI) — no extra library required.
    /// </summary>
    private async void OnNetworkTapped(object? sender, EventArgs e)
    {
        if (sender is not BindableObject b) return;
        if (b.BindingContext is not WifiNetworkItem item) return;

        var network = item.Network;

        if (network.IsOpen)
        {
            // Open network — connect directly without password prompt
            await _vm.ConnectToNetworkCommand.ExecuteAsync((network.Ssid, string.Empty));
        }
        else
        {
            // Native password prompt — no external library needed
            var password = await DisplayPromptAsync(
                title:       $"WiFi Password",
                message:     $"Enter password for \"{network.Ssid}\"",
                accept:      "Connect",
                cancel:      "Cancel",
                placeholder: "WiFi password",
                maxLength:   64,
                keyboard:    Keyboard.Default);

            if (password is null) return; // User cancelled

            await _vm.ConnectToNetworkCommand.ExecuteAsync((network.Ssid, password));
        }
    }
}
