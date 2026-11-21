// Technician Replay in a real browser, against the ground truth prepared by verify_real_replay.py (--browser-truth).
// usage: node replay_browser_check.cjs <truth.json> <out dir>
// env: WEB (default http://127.0.0.1:5174), CHROME (Chromium path), PW (playwright-core module path), TECH_USER, TECH_PASS
const fs = require('node:fs')
const path = require('node:path')
const { chromium } = require(process.env.PW || 'playwright-core')

const [, , truthPath, outDir] = process.argv
const truth = JSON.parse(fs.readFileSync(truthPath, 'utf8'))
const WEB = process.env.WEB || 'http://127.0.0.1:5174'
fs.mkdirSync(outDir, { recursive: true })

const results = []
const check = (name, ok, detail) => {
  results.push({ check: name, ok: !!ok, detail })
  console.log(`[${ok ? 'PASS' : 'FAIL'}] ${name}: ${detail}`)
}

// Ground truth helpers (same rules as the replay model, data from the core DB and the Recorder).
function mediaAt(t) {
  for (const s of truth.spans) if (t >= s.utcStartMs && t <= s.utcEndMs) return { assetId: s.assetId, sec: (s.mediaStartMs + (t - s.utcStartMs)) / 1000 }
  return null
}
function stepAt(t) {
  let best = null
  for (const s of truth.steps) if (s.startMs <= t && (s.endMs == null || t < s.endMs) && (!best || s.startMs >= best.startMs)) best = s
  return best
}
function poseAt(t) {
  const p = truth.points
  for (let i = 1; i < p.length; i++) {
    const [ta, xa, ya, , va] = p[i - 1]
    const [tb, xb, yb, , vb] = p[i]
    if (ta <= t && t <= tb && va && vb) {
      const f = tb === ta ? 0 : (t - ta) / (tb - ta)
      return [xa + (xb - xa) * f, ya + (yb - ya) * f]
    }
  }
  return null
}

;(async () => {
  const browser = await chromium.launch({ executablePath: process.env.CHROME || 'C:/Users/User/AppData/Local/ms-playwright/chromium-1248/chrome-win64/chrome.exe' })
  const page = await browser.newPage({ viewport: { width: 1600, height: 1000 } })
  const errors = []
  page.on('pageerror', (e) => errors.push(e.message))
  await page.goto(WEB + '/login')
  await page.locator('input').nth(0).fill(process.env.TECH_USER || 'technician')
  await page.locator('input[type=password]').fill(process.env.TECH_PASS || 'Technician@123')
  await page.keyboard.press('Enter')
  await page.waitForTimeout(1500)
  await page.goto(`${WEB}/tech/journals/${truth.journalId}`)
  await page.waitForSelector('[role=slider]', { timeout: 30000 })
  await page.waitForFunction(() => { const v = document.querySelector('video'); return v && v.readyState >= 2 }, null, { timeout: 60000 })
  check('ui.opens_as_technician', true, `${page.url()}`)

  const slider = page.locator('[role=slider]').first()
  const box = await slider.boundingBox()
  const from = Number(await slider.getAttribute('aria-valuemin'))
  const to = Number(await slider.getAttribute('aria-valuemax'))

  async function state() {
    return page.evaluate(() => {
      const v = document.querySelector('video')
      const s = document.querySelector('[data-testid=current-step]')
      const pos = [...document.querySelectorAll('dd, dl *')].map((e) => e.textContent || '').find((tx) => /^-?\d+\.\d{2}, -?\d+\.\d{2}/.test(tx.trim()))
      return {
        cursor: Number(document.querySelector('[role=slider]').getAttribute('aria-valuenow')),
        videoSec: v ? v.currentTime : null,
        ready: v ? v.readyState : null,
        paused: v ? v.paused : null,
        stepId: s ? s.getAttribute('data-step-id') : null,
        stepName: document.querySelector('[data-testid=current-step-name]')?.textContent || null,
        activeBlock: document.querySelector('[data-testid=step-lane] [data-active]')?.getAttribute('data-step-id') || null,
        position: pos ? pos.trim() : null,
      }
    })
  }

  async function seekTo(t, label) {
    const frac = (t - from) / (to - from)
    await page.mouse.click(box.x + Math.max(1, Math.min(box.width - 1, frac * box.width)), box.y + 15)
    await page.waitForTimeout(1800) // let the player seek and decode
    const s = await state()
    const media = mediaAt(s.cursor)
    const step = stepAt(s.cursor)
    const pose = poseAt(s.cursor)
    const videoErr = media && s.videoSec != null ? Math.abs(s.videoSec - media.sec) : null
    let posErr = null
    if (pose && s.position) {
      const [x, y] = s.position.split('·')[0].split(',').map(Number)
      posErr = Math.hypot(x - pose[0], y - pose[1])
    }
    const ok = (media == null || (videoErr != null && videoErr <= 0.5 && s.ready >= 2))
      && (step ? s.stepId === step.id && s.activeBlock === step.id : !s.stepId)
      && (posErr == null || posErr <= 0.06)
    const at = ((s.cursor - from) / 1000).toFixed(1)
    check(`seek.${label}`, ok, `t=${at}s video ${s.videoSec?.toFixed(2)}s (expected ${media ? media.sec.toFixed(2) : 'none'}, err ${videoErr?.toFixed(2)}s) · `
      + `step "${s.stepName}" ${step ? (s.stepId === step.id ? '= DB' : `!= DB ${step.name}`) : '(none)'}${s.activeBlock === (step && step.id) ? ', lane block lit' : ', lane block NOT lit'} · `
      + `map ${s.position} (trajectory ${pose ? pose.map((v) => v.toFixed(2)).join(', ') : '-'}, err ${posErr == null ? '-' : (posErr * 100).toFixed(1) + ' cm'})`)
    await page.screenshot({ path: path.join(outDir, `seek-${label}.png`) })
    return s
  }

  // Every step start (+0.5 s), the middle of every step, and second-by-second around a stop.
  for (const [i, st] of truth.steps.entries()) {
    await seekTo(st.startMs + 500, `step${i + 1}-start`)
    if (st.endMs) await seekTo((st.startMs + st.endMs) / 2, `step${i + 1}-middle`)
  }
  const move = truth.steps.find((s) => s.type === 'MOVE' && s.endMs && s.endMs - s.startMs > 8000)
  if (move) for (let k = 1; k <= 3; k++) await seekTo(move.startMs + k * 1000 + 2000, `move-second-${k}`)

  // Keyboard: arrows move the cursor by 5 s / 1 s, '.' jumps to the next step start.
  await seekTo(from + 10000, 'before-keys')
  await page.locator('body').click({ position: { x: 5, y: 5 } })
  let s0 = await state()
  await page.keyboard.press('ArrowRight'); await page.waitForTimeout(300)
  let s1 = await state()
  await page.keyboard.press('Shift+ArrowLeft'); await page.waitForTimeout(300)
  let s2 = await state()
  check('keys.arrows', Math.abs(s1.cursor - s0.cursor - 5000) < 5 && Math.abs(s1.cursor - s2.cursor - 1000) < 5,
    `→ moved ${(s1.cursor - s0.cursor) / 1000}s, Shift+← moved ${(s2.cursor - s1.cursor) / 1000}s`)
  await page.keyboard.press('.'); await page.waitForTimeout(1500)
  const s3 = await state()
  const next = truth.steps.map((s) => s.startMs).filter((x) => x > s2.cursor + 250).sort((a, b) => a - b)[0]
  check('keys.next_step', next != null && Math.abs(s3.cursor - next) < 300 && s3.stepId === stepAt(s3.cursor)?.id,
    `'.' went to ${((s3.cursor - from) / 1000).toFixed(1)}s, next step starts ${next ? ((next - from) / 1000).toFixed(1) : '-'}s, bar "${s3.stepName}"`)

  // Play 4 s at 1x: video and cursor advance together.
  const p0 = await state()
  await page.keyboard.press(' ')
  await page.waitForTimeout(4000)
  const p1 = await state()
  await page.keyboard.press(' ')
  const media1 = mediaAt(p1.cursor)
  const adv = (p1.cursor - p0.cursor) / 1000
  check('play.video_follows_cursor', !p1.paused && adv > 3 && adv < 5 && media1 && Math.abs(p1.videoSec - media1.sec) <= 0.5,
    `played ${adv.toFixed(2)}s of timeline; video at ${p1.videoSec?.toFixed(2)}s, expected ${media1?.sec.toFixed(2)}s`)

  // Vietnamese: the same page, step bar in Vietnamese.
  const vi = page.getByRole('button', { name: 'VI', exact: true })
  if (await vi.count()) {
    await vi.first().click(); await page.waitForTimeout(800)
    await seekTo(truth.steps[Math.min(2, truth.steps.length - 1)].startMs + 3000, 'vi')
    const label = await page.locator('[data-testid=current-step]').innerText()
    check('ui.vietnamese', /Đang ở bước/.test(label), label.replace(/\s+/g, ' ').slice(0, 120))
    await page.getByRole('button', { name: 'EN', exact: true }).first().click()
  }
  check('ui.no_page_errors', errors.length === 0, errors.slice(0, 3).join(' | ') || 'none')
  fs.writeFileSync(path.join(outDir, 'browser-report.json'), JSON.stringify(results, null, 1))
  const failed = results.filter((r) => !r.ok).length
  console.log(`\n${results.length - failed}/${results.length} browser checks passed`)
  await browser.close()
  process.exit(failed ? 1 : 0)
})().catch((e) => { console.error(e); process.exit(2) })
