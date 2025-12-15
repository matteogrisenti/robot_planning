#include "combinatorial_planning/approximate_cell_decomposition.h"
#include "combinatorial_planning/planning_utils.h" // For pointInPolygon helpers
#include <algorithm>
#include <cmath>
#include <iostream>
#include <ros/ros.h>


// Main Algorithm
std::shared_ptr<Roadmap> approximateCellDecomposition(const Map& map, int maxDepth, double minCellSize) {
    auto roadmap = std::make_shared<Roadmap>();
    roadmap->setMap(&map);
    std::vector<Cell> freeCells;

    // 1. Define the Root Cell (Map Bounding Box)
    double minX, minY, maxX, maxY;
    map.get_bounding_box(minX, minY, maxX, maxY);
    
    // Add a small buffer to ensure boundaries are covered
    Cell root(minX, minY, maxX, maxY);

    // 2. Perform Recursive Decomposition
    HelperAproximateCellDecomposition::recursiveDecomposition(root, map, 0, maxDepth, minCellSize, freeCells);
    roadmap->debugCells = std::make_shared<std::vector<Cell>>(freeCells);

    // 3. Convert Free Cells to Roadmap Nodes
    for (const auto& cell : freeCells) {
        // Check if the centroid of the cell is inside the map boundaries
        Vertex validNode = HelperAproximateCellDecomposition::calculateRefinedCentroid(cell, map);
        roadmap->addVertex(validNode);
    }

    // 4. Connect Adjacent Cells
    HelperAproximateCellDecomposition::connectAdjacentCells(freeCells, roadmap);

    return roadmap;
}



namespace HelperAproximateCellDecomposition {

    // Recursive Logic
    void recursiveDecomposition(const Cell& currentCell, const Map& map, 
                                int depth, int maxDepth, double minCellSize, 
                                std::vector<Cell>& freeCells) {
        
        // Check intersection with obstacles
        bool intersects = cellIntersectsObstacle(currentCell, map);

        // CASE 1: Completely Free
        // If it doesn't intersect any obstacle, it's free space. Keep it.
        if (!intersects) {
            freeCells.push_back(currentCell);
            return;
        }

        // CASE 2: Mixed/Occupied - Check termination criteria
        double width = currentCell.maxX - currentCell.minX;
        double height = currentCell.maxY - currentCell.minY;

        // If we reached max depth or cell is too small, we discard it (conservative approach: treat as obstacle)
        if (depth >= maxDepth || width <= minCellSize || height <= minCellSize) {
            return; 
        }

        // CASE 3: Mixed - Subdivide
        // Calculate Midpoints
        double midX = (currentCell.minX + currentCell.maxX) / 2.0;
        double midY = (currentCell.minY + currentCell.maxY) / 2.0;

        // Create 4 Children
        // Top-Left
        recursiveDecomposition(Cell(currentCell.minX, midY, midX, currentCell.maxY), 
                               map, depth + 1, maxDepth, minCellSize, freeCells);
        // Top-Right
        recursiveDecomposition(Cell(midX, midY, currentCell.maxX, currentCell.maxY), 
                               map, depth + 1, maxDepth, minCellSize, freeCells);
        // Bottom-Left
        recursiveDecomposition(Cell(currentCell.minX, currentCell.minY, midX, midY), 
                               map, depth + 1, maxDepth, minCellSize, freeCells);
        // Bottom-Right
        recursiveDecomposition(Cell(midX, currentCell.minY, currentCell.maxX, midY), 
                               map, depth + 1, maxDepth, minCellSize, freeCells);
    }

    
    
    // Helper: Intersection Check
    bool cellIntersectsObstacle(const Cell& cell, const Map& map) {
        // 1. Check if any Obstacle Vertex is inside the Cell.
        // 2. Check if the Cell Center is inside any Obstacle 
        //    ( This menage case wher the cell is fully inside a large obstacle).
        // 3. Check if Cell corners are in Obstacle.
        
        // 1. Check Obstacle Vertices inside Cell
        for (const auto& obs : map.obstacles.get_obstacles()) {
            for (const auto& p : obs.get_points()) {
                Vertex v(p.x, p.y);
                if (cell.contains(v)) return true;
            }
            
            // 2. Check Cell Vertices inside Obstacle (catches case where cell is fully inside a large obstacle)
            // We check center and corners
            Vertex corners[5] = {
                cell.center,
                {cell.minX, cell.minY}, {cell.maxX, cell.minY},
                {cell.maxX, cell.maxY}, {cell.minX, cell.maxY}
            };
            
            // 3. Check if Cell corners are in Obstacle.
            for(const auto& c : corners) {
                if(PlanningUtils::pointInObstacle(c, obs)) return true;
            }
        }
        
        return false;
    }




    Vertex calculateRefinedCentroid(const Cell& cell, const Map& map) {
        std::vector<Vertex> mapPoly;
        for(const auto& p : map.borders.get_points())
            mapPoly.push_back(PlanningUtils::toVertex(p));

        // 1. Check if the default center is valid (Inside Map)
        if (PlanningUtils::pointInPolygon(cell.center, mapPoly)) {
            return cell.center; 
        }
        ROS_INFO("Cell center (%f, %f) is outside map, refining centroid...", cell.center.x, cell.center.y);

        // 2. The Center is OUTSIDE. Compute the "Polygon of Overlap".
        // We collect all vertices that define the intersection polygon.
        std::vector<Vertex> overlapVertices;

        // A. Add Cell Corners that are inside the Map
        std::vector<Vertex> cellCorners = {
            {cell.minX, cell.minY}, {cell.maxX, cell.minY},
            {cell.maxX, cell.maxY}, {cell.minX, cell.maxY}
        };
        
        for (const auto& p : cellCorners) {
            if (PlanningUtils::pointInPolygon(p, mapPoly)) {
                overlapVertices.push_back(p);
                //debug ROS_INFO("Added corner (%f, %f) to overlapVertices", p.x, p.y);
            }
        }

        // B. Add Map Vertices that are inside the Cell
        for (const auto& p : mapPoly) {
            if (cell.contains(p)) {
                overlapVertices.push_back(p);
                //debug ROS_INFO("Added MAP corner (%f, %f) to overlapVertices", p.x, p.y);
            }
        }

        // C. Add Intersection Points between Cell Edges and Map Edges
        // Define Cell Segments
        std::vector<std::pair<Vertex, Vertex>> cellEdges;
        for (size_t i = 0; i < 4; ++i) {
            cellEdges.push_back({cellCorners[i], cellCorners[(i + 1) % 4]});
        }

        // Check against all Map Edges
        for (size_t i = 0; i < mapPoly.size(); ++i) {
            Vertex m1 = mapPoly[i];
            Vertex m2 = mapPoly[(i + 1) % mapPoly.size()];

            for (const auto& edge : cellEdges) {
                Vertex inter(0,0);
                if (PlanningUtils::getSegmentIntersection(edge.first, edge.second, m1, m2, inter)) {
                    // CHECK FOR DUPLICATES
                    if (!PlanningUtils::containsVertex(overlapVertices, inter)) {
                        overlapVertices.push_back(inter);
                        //debug ROS_INFO("Added intersection (%f, %f)", inter.x, inter.y);
                    }
                }
            }
        }

        // 3. Derive the Centroid of the Overlap
        if (overlapVertices.empty()) {
            // Fallback: This effectively means the cell is completely outside 
            // (should ideally be pruned earlier) or error. Return center as fail-safe.
            return cell.center;
        }

        double sumX = 0;
        double sumY = 0;
        for (const auto& v : overlapVertices) {
            sumX += v.x;
            sumY += v.y;
        }

        // Return the average of the vertices of the intersection polygon
        return Vertex(sumX / overlapVertices.size(), sumY / overlapVertices.size());
    }

    
    

    
    // Helper: Connectivity
    void connectAdjacentCells(const std::vector<Cell>& cells, std::shared_ptr<Roadmap> roadmap) {
        // Two rectangular cells are adjacent if they share a boundary.
        // We add a tiny epsilon to handle floating point inaccuracies.
        double eps = 1e-4;

        for (size_t i = 0; i < cells.size(); ++i) {
            for (size_t j = i + 1; j < cells.size(); ++j) {
                const Cell& c1 = cells[i];
                const Cell& c2 = cells[j];

                bool overlapX = (c1.minX < c2.maxX - eps) && (c1.maxX > c2.minX + eps);
                bool overlapY = (c1.minY < c2.maxY - eps) && (c1.maxY > c2.minY + eps);

                bool touchingX = std::abs(c1.maxX - c2.minX) < eps || std::abs(c1.minX - c2.maxX) < eps;
                bool touchingY = std::abs(c1.maxY - c2.minY) < eps || std::abs(c1.minY - c2.maxY) < eps;

                // Adjacent if they Overlap in one dimension AND Touch in the other
                if ((overlapX && touchingY) || (overlapY && touchingX)) {
                    roadmap->addEdge(i, j, true);
                }
            }
        }
    }

}





