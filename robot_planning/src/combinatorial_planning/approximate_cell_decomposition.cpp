#include "combinatorial_planning/approximate_cell_decomposition.h"
#include "combinatorial_planning/planning_utils.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace ApproximateDecomposition {

    Vertex toVertex(const Point& p) { return Vertex(p.x, p.y); }

    bool getSegmentIntersection(const Vertex& A, const Vertex& B, const Vertex& C, const Vertex& D, Vertex& intersection) {
        double a1 = B.y - A.y; double b1 = A.x - B.x; double c1 = a1 * A.x + b1 * A.y;
        double a2 = D.y - C.y; double b2 = C.x - D.x; double c2 = a2 * C.x + b2 * C.y;
        double det = a1 * b2 - a2 * b1;
        if (std::abs(det) < 1e-9) return false;
        Vertex p((b2 * c1 - b1 * c2) / det, (a1 * c2 - a2 * c1) / det);
        auto onSeg = [](const Vertex& p, const Vertex& s, const Vertex& e) {
            return p.x >= std::min(s.x, e.x) - 1e-7 && p.x <= std::max(s.x, e.x) + 1e-7 &&
                   p.y >= std::min(s.y, e.y) - 1e-7 && p.y <= std::max(s.y, e.y) + 1e-7;
        };
        if (onSeg(p, A, B) && onSeg(p, C, D)) { intersection = p; return true; }
        return false;
    }

    Vertex calculateRefinedCentroid(const Cell& cell, const Map& map) {
        std::vector<Vertex> mapPoly; for(const auto& p : map.borders.get_points()) mapPoly.push_back(toVertex(p));
        if (PlanningUtils::pointInPolygon(cell.center, mapPoly)) return cell.center; 
        
        std::vector<Vertex> ov;
        std::vector<Vertex> cc = {{cell.minX, cell.minY}, {cell.maxX, cell.minY}, {cell.maxX, cell.maxY}, {cell.minX, cell.maxY}};
        for (const auto& p : cc) if (PlanningUtils::pointInPolygon(p, mapPoly)) ov.push_back(p);
        for (const auto& p : mapPoly) if (cell.contains(p)) ov.push_back(p);
        
        std::vector<std::pair<Vertex, Vertex>> ce;
        for (size_t i = 0; i < 4; ++i) ce.push_back({cc[i], cc[(i + 1) % 4]});
        for (size_t i = 0; i < mapPoly.size(); ++i) {
            Vertex m1 = mapPoly[i], m2 = mapPoly[(i + 1) % mapPoly.size()];
            for (const auto& edge : ce) {
                Vertex inter(0,0);
                if (getSegmentIntersection(edge.first, edge.second, m1, m2, inter)) ov.push_back(inter);
            }
        }
        if (ov.empty()) return cell.center;
        double sx = 0, sy = 0; for (const auto& v : ov) { sx += v.x; sy += v.y; }
        return Vertex(sx / ov.size(), sy / ov.size());
    }

    bool cellIntersectsObstacle(const Cell& cell, const Map& map) {
        for (const auto& obs : map.obstacles.get_obstacles()) {
            for (const auto& p : obs.get_points()) if (cell.contains(Vertex(p.x, p.y))) return true;
            Vertex c[] = {cell.center, {cell.minX, cell.minY}, {cell.maxX, cell.minY}, {cell.maxX, cell.maxY}, {cell.minX, cell.maxY}};
            for(const auto& p : c) if(PlanningUtils::pointInObstacle(p, obs)) return true;
        }
        return false;
    }

    void connectAdjacentCells(const std::vector<Cell>& cells, std::shared_ptr<Roadmap> roadmap) {
        double eps = 1e-4;
        for (size_t i = 0; i < cells.size(); ++i) {
            for (size_t j = i + 1; j < cells.size(); ++j) {
                const Cell& c1 = cells[i]; const Cell& c2 = cells[j];
                bool oX = (c1.minX < c2.maxX - eps) && (c1.maxX > c2.minX + eps);
                bool oY = (c1.minY < c2.maxY - eps) && (c1.maxY > c2.minY + eps);
                bool tX = std::abs(c1.maxX - c2.minX) < eps || std::abs(c1.minX - c2.maxX) < eps;
                bool tY = std::abs(c1.maxY - c2.minY) < eps || std::abs(c1.minY - c2.maxY) < eps;
                if ((oX && tY) || (oY && tX)) roadmap->addEdge(i, j, true);
            }
        }
    }

    // INTEGRATION: Logica Targets
    bool cellContainsTarget(const Cell& cell, const Map& map, Vertex& outTarget) {
        Vertex s(map.start.get_position().x, map.start.get_position().y);
        if (cell.contains(s)) { outTarget = s; return true; }
        if (!map.gates.get_gates().empty()) {
            Point g = map.gates.get_gates()[0].get_position();
            Vertex gv(g.x, g.y);
            if (cell.contains(gv)) { outTarget = gv; return true; }
        }
        for (const auto& v : map.victims.get_victims()) {
            Point p = v.get_center();
            Vertex vv(p.x, p.y);
            if (cell.contains(vv)) { outTarget = vv; return true; }
        }
        return false;
    }

    void recursiveDecomposition(const Cell& currentCell, const Map& map, 
                                int depth, int maxDepth, double minCellSize, 
                                std::vector<Cell>& freeCells) {
        
        bool intersects = cellIntersectsObstacle(currentCell, map);
        Vertex dummy;
        bool hasTarget = cellContainsTarget(currentCell, map, dummy);

        // Se è libera E non ha target, allora è una foglia valida.
        // Se ha un target, continuiamo a dividere per raffinare la posizione.
        if (!intersects && !hasTarget) {
            freeCells.push_back(currentCell);
            return;
        }

        double width = currentCell.maxX - currentCell.minX;
        double height = currentCell.maxY - currentCell.minY;

        if (depth >= maxDepth || width <= minCellSize || height <= minCellSize) {
            // Se non interseca (ma magari aveva un target), la teniamo
            if (!intersects) freeCells.push_back(currentCell);
            return; 
        }

        double midX = (currentCell.minX + currentCell.maxX) / 2.0;
        double midY = (currentCell.minY + currentCell.maxY) / 2.0;

        recursiveDecomposition(Cell(currentCell.minX, midY, midX, currentCell.maxY), map, depth + 1, maxDepth, minCellSize, freeCells);
        recursiveDecomposition(Cell(midX, midY, currentCell.maxX, currentCell.maxY), map, depth + 1, maxDepth, minCellSize, freeCells);
        recursiveDecomposition(Cell(currentCell.minX, currentCell.minY, midX, midY), map, depth + 1, maxDepth, minCellSize, freeCells);
        recursiveDecomposition(Cell(midX, currentCell.minY, currentCell.maxX, midY), map, depth + 1, maxDepth, minCellSize, freeCells);
    }

    std::shared_ptr<Roadmap> approximateCellDecomposition(const Map& map, int maxDepth, double minCellSize) {
        auto roadmap = std::make_shared<Roadmap>();
        roadmap->setMap(&map);
        std::vector<Cell> freeCells;

        double minX, minY, maxX, maxY;
        map.get_bounding_box(minX, minY, maxX, maxY);
        Cell root(minX, minY, maxX, maxY);

        recursiveDecomposition(root, map, 0, maxDepth, minCellSize, freeCells);
        roadmap->debugCells = std::make_shared<std::vector<Cell>>(freeCells);

        for (const auto& cell : freeCells) {
            Vertex nodePos, targetPos;
            // INTEGRATION: Se c'è un target, il nodo è il target stesso
            if (cellContainsTarget(cell, map, targetPos)) nodePos = targetPos;
            else nodePos = calculateRefinedCentroid(cell, map);
            roadmap->addVertex(nodePos);
        }

        connectAdjacentCells(freeCells, roadmap);
        return roadmap;
    }
}