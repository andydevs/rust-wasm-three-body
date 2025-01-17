import { ThreeBodySystem } from "rust-wasm-three-body";
import { memory } from "rust-wasm-three-body/rust_wasm_three_body_bg.wasm"


/**
 * Random XY Vector
 */
function randomVec(maxRad) {
    let alpha = Math.PI*2*Math.random()
    let radius = maxRad*Math.random()
    let ux = Math.cos(alpha)
    let uy = Math.sin(alpha)
    return [ ux*radius, uy*radius ]
}

/**
 * JS representation of body
 */
class Body {
    constructor(radius, color, pos, vel) {
        this.radius = radius
        this.color = color
        this.pos = pos
        this.vel = vel
    }

    draw(ctx, origin) {
        let [ x, y ] = this.pos
        let [ ox, oy ] = origin
        ctx.fillStyle = this.color
        ctx.beginPath()
        ctx.arc(x + ox, y + oy, this.radius, 0, Math.PI*2)
        ctx.fill()
    }
}

// Initialize canvas
let canvas = document.getElementById('rw3b-canvas')
let ctx = canvas.getContext('2d')
let width
let height
let origin

let resize = (e) => {
    canvas.width = document.body.clientWidth
    canvas.height = document.body.clientHeight
    width = canvas.width
    height = canvas.height
    origin = [ width/2, height/2 ]
}
resize()


window.onresize = resize

// Initialize system and bodies
let system = ThreeBodySystem.new()
let radius = system.get_radius()
let bodies = [
    new Body(radius, '#ff0000', randomVec(300), randomVec(1.5)), 
    new Body(radius, '#ffff00', randomVec(300), randomVec(1.5)), 
    new Body(radius, '#00aaff', randomVec(300), randomVec(1.5))
]
system.initialize_position(...bodies.flatMap(b => b.pos))
system.initialize_velocity(...bodies.flatMap(b => b.vel))
let state = new Float32Array(memory.buffer, system.get_state(), system.get_state_size())

/**
 * Frame update. Calculate physics and draw bodies
 * 
 * @param {object} frame frame object
 */
function onFrame(frame) {
    // System update
    system.physics_update(frame.dt, frame.width, frame.height)
    
    // Get new positions
    bodies.forEach((body, i) => {
        body.pos = [ state[2*i], state[2*i+1] ]
    })

    // Draw bodies
    bodies.forEach(body => {
        body.draw(ctx, origin)
    })
}

/**
 * Animation function
 */
;(function animation() {
    let dt = 0.0
    let start = Date.now()
    function loop() {
        ctx.clearRect(0, 0, width, height)
        dt = Date.now() - start
        onFrame({ dt, width, height })
        start = Date.now()
        requestAnimationFrame(loop)
    }
    requestAnimationFrame(loop)
})()