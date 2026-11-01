# Niket.Pathfinding2D 0.1.0

First release of the precompiled DLLs for Unity. A node-graph A* pathfinder for 2D projects with Scene view tooling for building the graph.

## Contents

```
Niket.Pathfinding2D/
├── Niket.Pathfinding2D.dll          Runtime: Node, NodeParent, Pathfinder
└── Editor/
    └── Niket.Pathfinding2D.Editor.dll   Editor only: inspectors, gizmos, tangent handles
```

Both files are in [dist/Niket.Pathfinding2D](dist/Niket.Pathfinding2D).

## Requirements

- Unity 6 (built against 6000.3.11f1)
- .NET Standard 2.1 API compatibility level

## Install

1. Copy the `Niket.Pathfinding2D` folder into your project's `Assets` folder.
2. Keep the editor DLL inside the `Editor` subfolder so it is excluded from player builds.
3. If the project also contains the original `.cs` scripts, delete them. Otherwise Unity reports duplicate types.

Components attached from the source scripts will show as missing scripts. Re-attach them from the DLL.

## Quick start

1. Create an empty GameObject and add **NodeParent** (`Niket.Pathfinding2D`).
2. Click **Add Node** to create child nodes and position them.
3. Link them with **Auto Link By Radius**, or select several nodes and press **Link Selected Nodes**.
4. Find a path:

```csharp
using System.Collections.Generic;
using Niket.Pathfinding2D;
using UnityEngine;

public class Walker : MonoBehaviour
{
    [SerializeField] NodeParent graph;

    Pathfinder pathfinder;
    readonly List<Node> path = new List<Node>();

    void Start() => pathfinder = new Pathfinder(graph);

    public void GoTo(Vector2 target)
    {
        if (pathfinder.TryFindPath(transform.position, target, path, out float cost))
            foreach (Node n in path) Debug.Log(n.name);
    }
}
```

## Features in 0.1.0

- A* search with a binary-heap open set and reusable buffers
- Directional links with a cost multiplier (`cost = distance * weight`)
- Optional Bezier spline links with draggable tangent handles
- Auto-link by radius, manual linking, clear links
- Scene gizmos for nodes, links, direction arrows and cost labels
- Undo support for editor actions

## Known limitations

- `Pathfinder` is not thread safe; use one instance per thread.
- Nodes are collected on `Awake`; call `NodeParent.RefreshNodes()` after adding nodes at runtime.
- Link weights of 0 are allowed but reduce A* to Dijkstra.
- Not yet tested on other Unity versions or on device builds.

## Building from source

```
dotnet build Build/Niket.Pathfinding2D.Editor.csproj -c Release
```

The projects look for Unity in `C:\Program Files\Unity\Hub\Editor\6000.3.11f1`. For another install, add `-p:UnityEditorData="<path>\Editor\Data"`.
