import './styles/style.css'

const testBtn = document.getElementById('connectBtn') as HTMLButtonElement
const testOut = document.getElementById('connectOutput') as HTMLElement

const laserButton = document.getElementById('laserBtn') as HTMLButtonElement
const laserButtonOut = document.getElementById('laserOutput') as HTMLElement

testBtn.addEventListener('click', async () => {
  testOut.textContent = 'Connecting...'
  try {
    // Call the backend which is accessible on the host at port 443 in this compose setup
    const res = await fetch('http://localhost:443/media/mystream')
    const data = await res.json()
    testOut.textContent = JSON.stringify(data, null, 2)
  } catch (e) {
    testOut.textContent = 'Error: ' + String(e)
  }
})

laserButton.addEventListener('click', async () => {
  laserButtonOut.textContent = 'Connecting...'
  try {
    // Call the backend which is accessible on the host at port 443 in this compose setup
    const res = await fetch('http://localhost:443/media/mystream')
    const data = await res.json()
    laserButtonOut.textContent = JSON.stringify(data, null, 2)
  } catch (e) {
    laserButtonOut.textContent = 'Error: ' + String(e)
  }
})

