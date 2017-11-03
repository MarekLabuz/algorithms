const fs = require('fs')

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
        [v2]: { from: v1, to: v2, capacity }
      }
    }
    this.flows = {
      ...this.flows,
      [v1]: 0,
      [v2]: 0
    }
  }

  reset () {
    this.flows = Object.keys(this.flows).reduce((acc, id) => ({
      ...acc,
      [id]: 0
    }), {})
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
      const { capacity, to } = graph.vertexes[a][vertex]
      const flow = graph.flows[to]
      const cf = capacity - flow
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
  while (true) {
    const row = meta[state]
    if (row) {
      state = row[0]
      result.push(row[1])
    } else {
      break
    }
  }

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

      const { capacity, to } = graph.vertexes[u][v]
      const flow = graph.flows[to]
      const cf = capacity - flow

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
  do {
    const minCf = path.reduce((minCf, curr, i) => {
      if (!path[i - 1]) {
        return minCf
      }

      const { capacity, to } = graph.vertexes[path[i - 1]][curr]
      const flow = graph.flows[to]
      const cf = capacity - flow
      return cf < minCf ? cf : minCf
    }, Infinity)

    for (let i = 1; i < path.length; i += 1) {
      const edge = graph.vertexes[path[i - 1]][path[i]]
      edge.flow += minCf
      graph.flows[path[i - 1]] -= minCf
      graph.flows[path[i]] += minCf
    }

    graph.discovered.clear()
    path = algorithm(a, b)
  } while (path)
}

ff('10', '60', DFS)
console.log('DFS', graph.flows[60])

graph.reset()
ff('10', '60', BFS)
console.log('BFS', graph.flows[60])

// console.log(Object.keys(graph.vertexes[4]))
// console.log(BFS('10', '60'))

// console.log(Object.values(graph.vertexes[10]).reduce((acc, { to }) => {
//   return acc + graph.flows[to]
// }, 0))
