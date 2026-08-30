param([string]$Pi = "waro@warotrans.local")
ssh -t $Pi "~/warotrans_tools/rollback_source.sh"
if ($LASTEXITCODE -ne 0) { throw "Rollback failed with exit code $LASTEXITCODE" }
