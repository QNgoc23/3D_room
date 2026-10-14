const canvas = document.querySelector('#scene')
const ctx = canvas.getContext('2d')
const fixture = document.querySelector('#fixture')
const peoplePanel = document.querySelector('#people')
const invalidPanel = document.querySelector('#invalid')
const frameLabel = document.querySelector('#frameLabel')
const provenance = document.querySelector('#provenance')
const errorBox = document.querySelector('#error')
const playButton = document.querySelector('#play')

const state = { data: null, frame: 0, playing: true, timer: null }
const colors = ['#38bdf8', '#a78bfa', '#34d399', '#f472b6', '#facc15']

function assertContract(data) {
  if (data?.schema_version !== 'camera_aim_3d_vectors_v1') throw new Error('Sai schema_version')
  if (!Array.isArray(data.frames) || data.frames.length === 0) throw new Error('JSON không có frames')
  if (data.provenance?.source !== 'DEMO_SYNTHETIC') throw new Error('Viewer demo mặc định chỉ nhận DEMO_SYNTHETIC')
}

function resize() {
  const rect = canvas.getBoundingClientRect()
  const ratio = Math.min(window.devicePixelRatio || 1, 2)
  canvas.width = Math.round(rect.width * ratio)
  canvas.height = Math.round(rect.height * ratio)
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0)
  draw()
}

function projection() {
  const width = canvas.clientWidth
  const height = canvas.clientHeight
  const scale = Math.min(width / 10.5, height / 8.4)
  const origin = { x: width * 0.49, y: height * 0.28 }
  return (world) => {
    const sceneX = world.x - 2.25
    const sceneY = world.z || 0
    const sceneZ = 5 - world.y
    return {
      x: origin.x + (sceneX - sceneZ) * 0.74 * scale,
      y: origin.y + (sceneX + sceneZ) * 0.34 * scale - sceneY * scale,
    }
  }
}

function line(a, b, color, width = 1, dash = []) {
  const project = projection()
  const pa = project(a)
  const pb = project(b)
  ctx.beginPath()
  ctx.moveTo(pa.x, pa.y)
  ctx.lineTo(pb.x, pb.y)
  ctx.strokeStyle = color
  ctx.lineWidth = width
  ctx.setLineDash(dash)
  ctx.stroke()
  ctx.setLineDash([])
}

function drawRoom() {
  for (let x = 0; x <= 4.5 + 1e-9; x += 0.5) line({ x, y: 0, z: 0 }, { x, y: 10, z: 0 }, '#263650')
  for (let y = 0; y <= 10 + 1e-9; y += 0.5) line({ x: 0, y, z: 0 }, { x: 4.5, y, z: 0 }, '#263650')
  const corners = [[0, 0], [4.5, 0], [4.5, 10], [0, 10]]
  for (let i = 0; i < corners.length; i += 1) {
    const [x1, y1] = corners[i]
    const [x2, y2] = corners[(i + 1) % corners.length]
    line({ x: x1, y: y1, z: 0 }, { x: x2, y: y2, z: 0 }, '#8ba0bd', 1.6)
    line({ x: x1, y: y1, z: 3 }, { x: x2, y: y2, z: 3 }, '#33445f', 1)
    line({ x: x1, y: y1, z: 0 }, { x: x1, y: y1, z: 3 }, '#33445f', 1)
  }
  line({ x: 0, y: 0, z: 0 }, { x: 1.1, y: 0, z: 0 }, '#fb7185', 3)
  line({ x: 0, y: 0, z: 0 }, { x: 0, y: 1.4, z: 0 }, '#4ade80', 3)
  line({ x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 1.2 }, '#60a5fa', 3)
}

function drawTrajectory(person, color) {
  const points = person.trajectory || []
  if (points.length < 2) return
  const project = projection()
  ctx.beginPath()
  points.forEach((point, index) => {
    const p = project(point.position_world_m)
    if (index === 0) ctx.moveTo(p.x, p.y)
    else ctx.lineTo(p.x, p.y)
  })
  ctx.strokeStyle = `${color}aa`
  ctx.lineWidth = 2
  ctx.setLineDash([5, 5])
  ctx.stroke()
  ctx.setLineDash([])
}

function drawPerson(person, index) {
  if (!person.renderable || !person.position_world_m) return
  const color = person.trust_level === 'estimated' ? '#f59e0b' : colors[index % colors.length]
  drawTrajectory(person, color)
  const project = projection()
  const foot = project(person.position_world_m)
  const head = project({ ...person.position_world_m, z: 1.65 })
  ctx.beginPath()
  ctx.moveTo(foot.x, foot.y)
  ctx.lineTo(head.x, head.y)
  ctx.strokeStyle = color
  ctx.lineWidth = 10
  ctx.lineCap = 'round'
  ctx.stroke()
  ctx.beginPath()
  ctx.arc(head.x, head.y - 7, 9, 0, Math.PI * 2)
  ctx.fillStyle = color
  ctx.fill()
  ctx.beginPath()
  ctx.ellipse(foot.x, foot.y, 17, 8, 0, 0, Math.PI * 2)
  ctx.strokeStyle = color
  ctx.lineWidth = 2
  ctx.setLineDash(person.trust_level === 'estimated' ? [4, 3] : [])
  ctx.stroke()
  ctx.setLineDash([])
  ctx.fillStyle = '#f8fafc'
  ctx.font = '700 12px ui-monospace, monospace'
  ctx.fillText(person.id, head.x + 14, head.y - 9)
}

function card(person, kind) {
  const element = document.createElement('div')
  element.className = `person ${kind}`
  const title = document.createElement('strong')
  title.textContent = person.id
  const detail = document.createElement('span')
  const world = person.position_world_m
  detail.textContent = world
    ? `World (${world.x.toFixed(2)}, ${world.y.toFixed(2)}, ${world.z.toFixed(2)}) · ${person.grid_cell.cell_id} · ${person.trust_level}`
    : `${person.position_status} · ${person.position_reason}`
  element.append(title, detail)
  return element
}

function updatePanels(frame) {
  peoplePanel.replaceChildren()
  invalidPanel.replaceChildren()
  const valid = frame.people.filter((person) => person.renderable)
  const invalid = frame.people.filter((person) => !person.renderable)
  valid.forEach((person) => peoplePanel.append(card(person, person.trust_level === 'estimated' ? 'warn' : '')))
  invalid.forEach((person) => invalidPanel.append(card(person, 'invalid')))
  if (!valid.length) peoplePanel.textContent = 'Không có vector có thể đặt trong scene.'
  if (!invalid.length) invalidPanel.textContent = 'Không có.'
}

function draw() {
  const width = canvas.clientWidth
  const height = canvas.clientHeight
  ctx.clearRect(0, 0, width, height)
  const gradient = ctx.createLinearGradient(0, 0, 0, height)
  gradient.addColorStop(0, '#111a2c')
  gradient.addColorStop(1, '#070b13')
  ctx.fillStyle = gradient
  ctx.fillRect(0, 0, width, height)
  drawRoom()
  if (!state.data) return
  const frame = state.data.frames[state.frame]
  frame.people.forEach(drawPerson)
  frameLabel.textContent = `${frame.frame_id} · ${state.frame + 1}/${state.data.frames.length}`
  updatePanels(frame)
}

function step(delta) {
  if (!state.data) return
  state.frame = (state.frame + delta + state.data.frames.length) % state.data.frames.length
  draw()
}

function schedule() {
  clearInterval(state.timer)
  if (state.playing) state.timer = setInterval(() => step(1), 900)
  playButton.textContent = state.playing ? 'Tạm dừng' : 'Phát'
}

async function load(url) {
  try {
    errorBox.hidden = true
    const response = await fetch(url, { cache: 'no-store' })
    if (!response.ok) throw new Error(`HTTP ${response.status}`)
    const data = await response.json()
    assertContract(data)
    state.data = data
    state.frame = 0
    provenance.textContent = 'DEMO / SIMULATED · KHÔNG PHẢI LIVE'
    draw()
    schedule()
  } catch (error) {
    state.data = null
    errorBox.hidden = false
    errorBox.textContent = `Không tải được fixture: ${error instanceof Error ? error.message : error}`
    draw()
  }
}

document.querySelector('#previous').addEventListener('click', () => step(-1))
document.querySelector('#next').addEventListener('click', () => step(1))
playButton.addEventListener('click', () => { state.playing = !state.playing; schedule() })
fixture.addEventListener('change', () => load(fixture.value))
window.addEventListener('resize', resize)

resize()
load(fixture.value)
