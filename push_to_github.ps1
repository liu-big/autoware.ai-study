# 推送到 GitHub 私有仓库（liu-big）
# 用法：在 PowerShell 中执行
#   .\push_to_github.ps1
# 要求：
#   1. 已安装 Git
#   2. 浏览器能登录 https://github.com/liu-big
#   3. 已创建 Personal Access Token（classic）并勾选 repo 权限
#      创建地址：https://github.com/settings/tokens
#   4. 本机可访问 api.github.com（若公司网络拦截，请换网络/关代理后重试）

$ErrorActionPreference = "Stop"
$RepoName = "autoware.ai-study"
$Owner = "liu-big"
$Email = "2715209705@qq.com"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $Root

Write-Host "==> 工作目录: $Root"

# 优先使用已下载的 gh
$gh = "C:\KeilTemp\gh-cli\bin\gh.exe"
if (-not (Test-Path $gh)) {
  $gh = (Get-Command gh -ErrorAction SilentlyContinue).Source
}
if (-not $gh) {
  Write-Host "未找到 gh.exe。请先安装 GitHub CLI: https://cli.github.com/" -ForegroundColor Red
  exit 1
}

Write-Host "==> 使用 gh: $gh"

# 登录检查
& $gh auth status 2>$null
if ($LASTEXITCODE -ne 0) {
  Write-Host ""
  Write-Host "尚未登录。请粘贴你的 GitHub Token（不会显示在屏幕上）:" -ForegroundColor Yellow
  Write-Host "  创建 Token: https://github.com/settings/tokens  (勾选 repo)" -ForegroundColor Yellow
  $token = Read-Host -AsSecureString "PAT"
  $BSTR = [System.Runtime.InteropServices.Marshal]::SecureStringToBSTR($token)
  $plain = [System.Runtime.InteropServices.Marshal]::PtrToStringAuto($BSTR)
  $plain | & $gh auth login --with-token
  if ($LASTEXITCODE -ne 0) {
    Write-Host "登录失败，请检查 Token 与网络。" -ForegroundColor Red
    exit 1
  }
}

$user = (& $gh api user --jq .login).Trim()
Write-Host "==> 当前登录账号: $user"
if ($user -ne $Owner) {
  Write-Host "警告: 当前不是 $Owner，将推到当前账号 $user 下。" -ForegroundColor Yellow
  $Owner = $user
}

# 创建私有仓库（已存在则跳过）
$full = "$Owner/$RepoName"
$exists = $true
& $gh repo view $full 2>$null | Out-Null
if ($LASTEXITCODE -ne 0) { $exists = $false }

if (-not $exists) {
  Write-Host "==> 创建私有仓库 $full ..."
  & $gh repo create $full --private --source=. --remote=origin --description "Autoware.ai 实车定制栈（Ant）- 仅所有者可访问"
  if ($LASTEXITCODE -ne 0) {
    Write-Host "创建失败。" -ForegroundColor Red
    exit 1
  }
} else {
  Write-Host "==> 仓库已存在，设置 remote ..."
  $url = "https://github.com/$full.git"
  $remotes = git remote
  if ($remotes -notcontains "origin") {
    git remote add origin $url
  } else {
    git remote set-url origin $url
  }
}

Write-Host "==> 推送 main ..."
git push -u origin main
if ($LASTEXITCODE -ne 0) {
  Write-Host "推送失败。可尝试: git push -u origin main" -ForegroundColor Red
  exit 1
}

Write-Host ""
Write-Host "完成。私有仓库地址:" -ForegroundColor Green
Write-Host "  https://github.com/$full"
Write-Host "请确认 Settings → Collaborators 中没有其他人。"
Write-Host "提交作者邮箱应为: $Email"
