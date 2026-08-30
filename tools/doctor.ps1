# Run the Pi-side doctor script from Windows after at least one deploy copied the tools.
param(
    [string]$Pi = "waro@warotrans.local",
    [ValidateSet("idle", "base", "mapping", "navigation")]
    [string]$Mode = "idle"
)

ssh $Pi "~/warotrans_tools/doctor.sh $Mode"
