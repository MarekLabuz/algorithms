const fs = require('fs')

class Graph {
  constructor () {
    this.nodes = {}
    this.edges = {}
  }

  addNode (source, params = {}) {
    this.nodes[source] = { id: source, ...params }
  }

  addEdge (source, dest, params = {}) {
    this.edges[source] = {
      ...this.edges[source],
      [dest]: {
        ...(this.edges[source] || {})[dest],
        ...params
      }
    }
  }

  getNeighbours (id) {
    return Object.keys(this.edges[id])
  }

  updateNode (id, params) {
    this.nodes[id] = { ...this.nodes[id], ...params }
  }
}

const graph = new Graph()
const initialNode = 1
const goalNode = 20

fs.readFileSync('graf.txt', 'utf-8')
  .split('\r\n')
  .map(v => v.split(';').map(e => parseInt(e)))
  .forEach(([source, dest, weight]) => {
    graph.addNode(source, { dist: Infinity, visited: false, path: [source] })
    graph.addNode(dest, { dist: Infinity, visited: false, path: [dest] })
    graph.addEdge(source, dest, { weight })
  })

const dijkstra = (currNode) => {
  graph.getNeighbours(currNode)
    .forEach((neighbourNode) => {
      const edge = graph.edges[currNode][neighbourNode].weight
      graph.updateNode(neighbourNode, {
        dist: graph.nodes[currNode].dist + edge,
        path: [...graph.nodes[currNode].path, neighbourNode]
      })
    })

  graph.nodes[currNode].visited = true

  const { id: node, dist } = Object.keys(graph.nodes)
    .reduce(
      (acc, currNode) => (
        !graph.nodes[currNode].visited && graph.nodes[currNode].dist < acc.dist ? graph.nodes[currNode] : acc
      ),
      { dist: Infinity }
    )

  return (
    currNode === goalNode || dist === Infinity
      ? graph.nodes[currNode]
      : dijkstra(node)
  )
}

graph.nodes[initialNode].dist = 0
const { dist, path } = dijkstra(initialNode)
console.log(`Distance: ${dist}`)
console.log(`Path: ${path}`)
