namespace RisaBotApp.Pages;

/// <summary>
/// Splash screen: shows RISA-Bot branding for 1.8s then navigates to DiscoveryPage.
/// No network calls happen here — kept intentionally lightweight for fast perceived startup.
/// </summary>
public partial class SplashPage : ContentPage
{
    public SplashPage()
    {
        InitializeComponent();
    }

    protected override async void OnAppearing()
    {
        base.OnAppearing();

        // 1.8s brand moment, then hand off to Discovery
        await Task.Delay(1800);
        await Shell.Current.GoToAsync("//DiscoveryPage");
    }
}
