#include "libraries/combinatorial_planning/approximate_cell_decomposition.h"
#include "libraries/planning_utils.h" 
#include <algorithm>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <limits>

namespace HelperAproximateCellDecomposition {
    // Forward declaration 
    void recursiveDecomposition(const Cell& currentCell, const Map& map, 
                                int depth, int maxDepth, double minCellSize, 
                                std::vector<Cell>& freeCells,
                                const std::vector<Point>& targets);
    
    // Altre forward declaration corrette
    bool cellIntersectsObstacle(const Cell& cell, const Map& map);
    bool cellOutsideCheck(const Cell& cell, const Map& map);
    Vertex calculateRefinedCentroid(const Cell& cell, const Map& map);
    // Firma corretta per accettare la mappa dei nodi validi
    void connectAdjacentCells(const std::vector<Cell>& cells, std::shared_ptr<Roadmap> roadmap, const std::vector<int>& mapping);
}

// Main Algorithm
std::shared_ptr<Roadmap> approximateCellDecomposition(const Map& map, int maxDepth, double minCellSize) {
    auto roadmap = std::make_shared<Roadmap>();
    roadmap->setMap(&map);
    std::vector<Cell> freeCells;

    // --- 0. PREPARAZIONE TARGETS ---
    std::vector<Point> targets;
    if (!map.gates.get_gates().empty()) {
        targets.push_back(map.gates.get_gates()[0].get_position());
    }
    for (const auto& v : map.victims.get_victims()) {
        targets.push_back(v.get_center());
    }

    // 1. Define the Root Cell
    double minX, minY, maxX, maxY;
    map.get_bounding_box(minX, minY, maxX, maxY);
    Cell root(minX, minY, maxX, maxY);

    // 2. Perform Recursive Decomposition
    HelperAproximateCellDecomposition::recursiveDecomposition(root, map, 0, maxDepth, minCellSize, freeCells, targets);
    roadmap->debugCells = std::make_shared<std::vector<Cell>>(freeCells);

    // 3. Convert Free Cells to Roadmap Nodes
    std::vector<int> cellIndexToNodeId(freeCells.size(), -1);
    double safety_margin = 0.35; // Margine di sicurezza

    for (size_t i = 0; i < freeCells.size(); ++i) {
        Vertex nodePos = HelperAproximateCellDecomposition::calculateRefinedCentroid(freeCells[i], map);
        
        // Sovrapposizione Target
        for (const auto& t : targets) {
            if (freeCells[i].contains(Vertex(t.x, t.y))) {
                nodePos = Vertex(t.x, t.y);
                break; 
            }
        }

        // QUI IL CHECK DI SICUREZZA (Correttamente posizionato)
        if (PlanningUtils::isPointValid(nodePos.x, nodePos.y, map, safety_margin)) {
            cellIndexToNodeId[i] = roadmap->addVertex(nodePos);
        }
    }

    // 4. Connect Adjacent Cells (passando la mapping)
    HelperAproximateCellDecomposition::connectAdjacentCells(freeCells, roadmap, cellIndexToNodeId);

    return roadmap;
}

namespace HelperAproximateCellDecomposition {

    void recursiveDecomposition(const Cell& currentCell, const Map& map, 
                                int depth, int maxDepth, double minCellSize, 
                                std::vector<Cell>& freeCells,
                                const std::vector<Point>& targets) {
        
        bool intersects = cellIntersectsObstacle(currentCell, map);
        bool fullyOutside = cellOutsideCheck(currentCell, map);
        
        if (fullyOutside) return; 

        bool containsTarget = false;
        for(const auto& t : targets) {
            if (currentCell.contains(Vertex(t.x, t.y))) {
                containsTarget = true;
                break;
            }
        }

        if (!intersects && !containsTarget) {
            freeCells.push_back(currentCell);
            return;
        }

        double width = currentCell.maxX - currentCell.minX;
        double height = currentCell.maxY - currentCell.minY;

        if (depth >= maxDepth || width <= minCellSize || height <= minCellSize) {
            if (!intersects || containsTarget) {
                 if (!intersects) freeCells.push_back(currentCell);
            }
            return; 
        }

        double midX = (currentCell.minX + currentCell.maxX) / 2.0;
        double midY = (currentCell.minY + currentCell.maxY) / 2.0;

        recursiveDecomposition(Cell(currentCell.minX, midY, midX, currentCell.maxY), map, depth + 1, maxDepth, minCellSize, freeCells, targets);
        recursiveDecomposition(Cell(midX, midY, currentCell.maxX, currentCell.maxY), map, depth + 1, maxDepth, minCellSize, freeCells, targets);
        recursiveDecomposition(Cell(currentCell.minX, currentCell.minY, midX, midY), map, depth + 1, maxDepth, minCellSize, freeCells, targets);
        recursiveDecomposition(Cell(midX, currentCell.minY, currentCell.maxX, midY), map, depth + 1, maxDepth, minCellSize, freeCells, targets);
    }

    bool cellIntersectsObstacle(const Cell& cell, const Map& map) {
        for (const auto& obs : map.obstacles.get_obstacles()) {
            for (const auto& p : obs.get_points()) {
                Vertex v(p.x, p.y);
                if (cell.contains(v)) return true;
            }
            Vertex corners[5] = {
                cell.center,
                {cell.minX, cell.minY}, {cell.maxX, cell.minY},
                {cell.maxX, cell.maxY}, {cell.minX, cell.maxY}
            };
            for(const auto& c : corners) {
                if(PlanningUtils::pointInObstacle(c, obs)) return true;
            }
        }
        return false;
    }

    bool cellOutsideCheck(const Cell& cell, const Map& map) {
        Vertex corners[4] = {
            {cell.minX, cell.minY}, {cell.maxX, cell.minY},
            {cell.maxX, cell.maxY}, {cell.minX, cell.maxY}
        };

        std::vector<Vertex> mapPoly;
        for(const auto& p : map.borders.get_points())
            mapPoly.push_back(PlanningUtils::toVertex(p));

        for (const Vertex corner : corners) {
            if (PlanningUtils::pointInPolygon(corner, mapPoly)) return false; 
        }
        for (const Vertex mapCorner : mapPoly) {
            if (mapCorner.x >= cell.minX && mapCorner.x <= cell.maxX &&
                mapCorner.y >= cell.minY && mapCorner.y <= cell.maxY) {
                return false; 
            }
        }
        return true; 
    }

    Vertex calculateRefinedCentroid(const Cell& cell, const Map& map) {
        std::vector<Vertex> mapPoly;
        for(const auto& p : map.borders.get_points())
            mapPoly.push_back(PlanningUtils::toVertex(p));

        if (PlanningUtils::pointInPolygon(cell.center, mapPoly)) return cell.center; 

        std::vector<Vertex> overlapVertices;
        std::vector<Vertex> cellCorners = {
            {cell.minX, cell.minY}, {cell.maxX, cell.minY},
            {cell.maxX, cell.maxY}, {cell.minX, cell.maxY}
        };
        
        for (const auto& p : cellCorners) {
            if (PlanningUtils::pointInPolygon(p, mapPoly)) overlapVertices.push_back(p);
        }
        if (overlapVertices.empty()) return cell.center;
        
        double sumX = 0, sumY = 0;
        for (const auto& v : overlapVertices) { sumX += v.x; sumY += v.y; }
        return Vertex(sumX / overlapVertices.size(), sumY / overlapVertices.size());
    }

    void connectAdjacentCells(const std::vector<Cell>& cells, std::shared_ptr<Roadmap> roadmap, const std::vector<int>& mapping) {
        double eps = 1e-4;
        for (size_t i = 0; i < cells.size(); ++i) {
            if (mapping[i] == -1) continue; // SKIP NODI INVALIDI

            for (size_t j = i + 1; j < cells.size(); ++j) {
                if (mapping[j] == -1) continue; // SKIP NODI INVALIDI

                const Cell& c1 = cells[i];
                const Cell& c2 = cells[j];

                bool overlapX = (c1.minX < c2.maxX - eps) && (c1.maxX > c2.minX + eps);
                bool overlapY = (c1.minY < c2.maxY - eps) && (c1.maxY > c2.minY + eps);
                bool touchingX = std::abs(c1.maxX - c2.minX) < eps || std::abs(c1.minX - c2.maxX) < eps;
                bool touchingY = std::abs(c1.maxY - c2.minY) < eps || std::abs(c1.minY - c2.maxY) < eps;

                if ((overlapX && touchingY) || (overlapY && touchingX)) {
                    roadmap->addEdge(mapping[i], mapping[j], true);
                }
            }
        }
    }
}