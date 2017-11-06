const fs = require('fs')

const log = require('single-line-log').stdout

class Graph {
  constructor () {
    this.vertexes = {}
    this.discovered = new Set()
    this.flows = {}
  }

  addEdge (v1, v2, capacity) {
    this.vertexes = {
      ...this.vertexes,
      [v1]: {
        ...this.vertexes[v1],
        [v2]: { capacity }
      }
    }
  }

  setFlow (v1, v2, value) {
    if (!this.flows[v1]) {
      this.flows[v1] = {}
    }
    this.flows[v1][v2] = value
  }

  getFlow (v1, v2) {
    return this.flows[v1] && this.flows[v1][v2] || 0
  }

  reset () {
    this.flows = {}
    this.discovered.clear()
  }
}

const graph = new Graph()

const DFS = (a, b, path = [a]) => {
  graph.discovered.add(a)
  if (a === b) {
    return path
  } else {
    const connectedVertexes = Object.keys(graph.vertexes[a])
    for (const vertex of connectedVertexes) {
      const { capacity } = graph.vertexes[a][vertex]
      const cf = capacity - graph.getFlow(a, vertex)
      if (!graph.discovered.has(vertex) && cf > 0) {
        const p = DFS(vertex, b, [...path, vertex])
        if (p) {
          return p
        }
      }
    }
  }
  return null
}

const constructPath = (s, meta) => {
  const result = [s]

  let state = s
  let row
  do {
    row = meta[state]
    if (row) {
      state = row[0]
      result.push(row[1])
    }
  } while (row)

  return result.reverse().slice(1)
}

const BFS = (a, b) => {
  const queue = []
  graph.discovered.clear()
  const meta = {}

  meta[a] = [null, null]
  queue.push(a)

  while (queue.length) {
    const u = queue.shift()

    if (u === b) {
      return constructPath(u, meta)
    }

    const connectedVertexes = Object.keys(graph.vertexes[u])
    for (const v of connectedVertexes) {
      if (graph.discovered.has(v)) {
        continue
      }

      const { capacity } = graph.vertexes[u][v]
      const cf = capacity - graph.getFlow(u, v)

      if (!queue.includes(v) && cf > 0) {
        meta[v] = [u, u]
        queue.push(v)
      }
    }

    graph.discovered.add(u)
  }
}

fs.readFileSync('graf1.txt', 'utf-8')
  .split('\n')
  .filter(v => v)
  .map(v => v.split(';').map(e => parseFloat(e)))
  .forEach(([v1, v2, capacity]) => {
    graph.addEdge(v1, v2, capacity)
  })

const ff = (a, b, algorithm) => {
  let path = algorithm(a, b)
  while (path) {
    const minCf = path.reduce((minCf, curr, i) => {
      if (!path[i - 1]) {
        return minCf
      }
      const { capacity } = graph.vertexes[path[i - 1]][curr]
      const cf = capacity - graph.getFlow(path[i - 1], curr)
      return cf < minCf ? cf : minCf
    }, Infinity)

    for (let i = 1; i < path.length; i += 1) {
      const flow = graph.getFlow(path[i - 1], path[i])
      graph.setFlow(path[i - 1], path[i], flow + minCf)
    }

    graph.discovered.clear()
    path = algorithm(a, b)
  }
}

const sumFlow = id => Object.values(graph.flows[id])
  .reduce((acc, flow) => acc + flow, 0)

ff('10', '60', DFS)
console.log('DFS', sumFlow('10'))

graph.reset()
ff('10', '60', BFS)
console.log('BFS', sumFlow('10'))

let maxFlow = 0
let vertexWithMaxFlow = '10'
for (let i = 1; i <= 99; i += 1) {
  if (i !== 10) {
    graph.reset()
    ff('10', `${i}`, DFS)
    const flow = sumFlow('10')
    if (flow > maxFlow) {
      maxFlow = flow
      vertexWithMaxFlow = i
    }
    log(`${i}/99`, flow)
  }
}

console.log('\n', vertexWithMaxFlow, maxFlow)
