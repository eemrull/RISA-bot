using RisaBotApp.ViewModels;

namespace RisaBotApp.Pages;

public partial class DiscoveryPage : ContentPage
{
    private readonly DiscoveryViewModel _vm;

    public DiscoveryPage(DiscoveryViewModel vm)
    {
        InitializeComponent();
        _vm = vm;
        BindingContext = vm;
    }

    protected override async void OnAppearing()
    {
        base.OnAppearing();
        // Trigger parallel network discovery scan to populate NoMachine device choice grid
        await _vm.RunNetworkScanAsync();
    }
}
