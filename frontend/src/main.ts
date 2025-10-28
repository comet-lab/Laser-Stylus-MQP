import './styles/style.css'

const btn = document.getElementById('connectBtn') as HTMLButtonElement
const out = document.getElementById('output') as HTMLElement

btn.addEventListener('click', async () => {
  out.textContent = 'Connecting...'
  try {
    // Call the backend which is accessible on the host at port 443 in this compose setup
    const res = await fetch('http://localhost:443/media/mystream')
    const data = await res.json()
    out.textContent = JSON.stringify(data, null, 2)
  } catch (e) {
    out.textContent = 'Error: ' + String(e)
  }
})
