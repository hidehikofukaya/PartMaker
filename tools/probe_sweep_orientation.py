"""Oracle for the bead-sweep twist: reproduce Gemini's multi-section MakePipeShell on an
S-shaped / C-shaped fold chain with an ASYMMETRIC (bead) profile and detect a 180-degree flip
by measuring where the bead top ends up relative to the intended panel normal."""
import math, warnings
warnings.simplefilter("ignore")
from OCC.Core.gp import gp_Pnt, gp_Dir, gp_Vec, gp_Ax1, gp_Ax3, gp_Trsf
from OCC.Core.BRepBuilderAPI import (BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire,
                                     BRepBuilderAPI_Transform, BRepBuilderAPI_MakeVertex)
from OCC.Core.BRepOffsetAPI import BRepOffsetAPI_MakePipeShell
from OCC.Core.GC import GC_MakeArcOfCircle
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.TopoDS import topods
from OCC.Core.GeomAbs import GeomAbs_Plane, GeomAbs_Cylinder, GeomAbs_BSplineSurface
from OCC.Core.BRepCheck import BRepCheck_Analyzer

R, L, W, DEPTH = 15.0, 60.0, 20.0, 6.0

def vadd(a, b, k=1.0): return (a[0]+k*b[0], a[1]+k*b[1], a[2]+k*b[2])
def cross(a, b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def dot(a, b): return sum(x*y for x, y in zip(a, b))
def norm(a):
    l = math.sqrt(dot(a, a)); return tuple(x/l for x in a)
def rot(v, axis, ang):
    """Rodrigues rotation of v about unit axis."""
    c, s = math.cos(ang), math.sin(ang)
    k = axis
    return tuple(v[i]*c + cross(k, v)[i]*s + k[i]*dot(k, v)*(1-c) for i in range(3))

def chain(a1, a2, tilt_deg):
    """Fold chain: line, arc(a1), line, arc(a2), line. Returns components with s-range, and per-panel normals.
    tilt rotates the 2nd fold axis about the path tangent (skew bend => non-planar spine)."""
    comps, normals = [], []
    p, t, n, s = (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), 0.0
    def line(p, t):
        nonlocal s
        q = vadd(p, t, L)
        comps.append(dict(type="line", edge=BRepBuilderAPI_MakeEdge(gp_Pnt(*p), gp_Pnt(*q)).Edge(), s0=s, s1=s+L))
        s += L; return q
    def arc(p, t, n, ang):
        nonlocal s
        sg = 1 if ang > 0 else -1; a = abs(math.radians(ang))
        axis = norm(cross(t, n)) if sg > 0 else norm(cross(n, t))  # rotate t toward +n*sg
        c = vadd(p, n, R*sg)
        def pt(th):
            r0 = vadd((0, 0, 0), n, -R*sg)
            return vadd(c, rot(r0, axis, th))
        q, m = pt(a), pt(a/2)
        comps.append(dict(type="arc", edge=BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(gp_Pnt(*p), gp_Pnt(*m), gp_Pnt(*q)).Value()).Edge(), s0=s, s1=s+R*a))
        s += R*a
        return q, rot(t, axis, a), rot(n, axis, a)
    normals.append(n); p = line(p, t)
    p, t, n = arc(p, t, n, a1); normals.append(n); p = line(p, t)
    if tilt_deg:
        n = rot(n, t, math.radians(tilt_deg))
    p, t, n = arc(p, t, n, a2); normals.append(n); p = line(p, t)
    return comps, normals, s

def eval_pt(comps, s):
    for c in comps:
        if c["s0"]-1e-6 <= s <= c["s1"]+1e-6:
            cv = BRepAdaptor_Curve(c["edge"]); u0, u1 = cv.FirstParameter(), cv.LastParameter()
            u = u0 + (u1-u0)*max(0, min(1, (s-c["s0"])/max(1e-9, c["s1"]-c["s0"])))
            P, V = gp_Pnt(), gp_Vec(); cv.D1(u, P, V)
            return (P.X(), P.Y(), P.Z()), norm((V.X(), V.Y(), V.Z()))
    raise ValueError(s)

def eval_n(comps, normals, s):
    for i, c in enumerate(comps):
        if c["s0"]-1e-6 <= s <= c["s1"]+1e-6:
            if c["type"] == "line": return normals[i//2]
            n1, n2 = normals[i//2], normals[i//2+1]; f = (s-c["s0"])/(c["s1"]-c["s0"])
            return norm(tuple(n1[j]+f*(n2[j]-n1[j]) for j in range(3)))
    raise ValueError(s)

def profile(depth, bead_only=False):
    """In the source frame (Gemini's): N=(1,0,0) is the tangent, X=(0,1,0) width, Z=(0,0,1) normal."""
    ys = [-6, -3, 3, 6]; zs = [0, depth, depth, 0]
    pts = [(0, -W, 0)] if not bead_only else []
    pts += [(0, y, z) for y, z in zip(ys, zs)]
    if not bead_only: pts.append((0, W, 0))
    mk = BRepBuilderAPI_MakeWire()
    for a, b in zip(pts[:-1], pts[1:]):
        mk.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(*a), gp_Pnt(*b)).Edge())
    return mk.Wire()

def orient_buggy(wire, origin, tangent, normal):
    """gsd_build._orient_profile_to_frame as of 2026-09-03: lays the section in the plane containing the tangent."""
    width = norm(cross(normal, tangent))
    src = gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0), gp_Dir(0, 1, 0))
    dst = gp_Ax3(gp_Pnt(*origin), gp_Dir(*tangent), gp_Dir(*width))
    tr = gp_Trsf(); tr.SetTransformation(dst, src)
    return topods.Wire(BRepBuilderAPI_Transform(wire, tr, True).Shape())

def orient(wire, origin, tangent, normal):
    """Correct placement: profile modelled with x=tangent (section plane x=0), y=width, z=normal."""
    dst = gp_Ax3(gp_Pnt(*origin), gp_Dir(*normal), gp_Dir(*tangent))   # Z=normal, X=tangent, Y=normal x tangent=width
    tr = gp_Trsf(); tr.SetTransformation(dst, gp_Ax3())                  # dst-relative coords -> global
    return topods.Wire(BRepBuilderAPI_Transform(wire, tr, True).Shape())

def spine_wire(comps, s_cuts=None):
    mk = BRepBuilderAPI_MakeWire()
    for c in comps: mk.Add(c["edge"])
    return mk.Wire()

def sub_wire(comps, s_a, s_b):
    """Trim the chain to [s_a, s_b] (line/arc exact), like Gemini's middle section."""
    mk = BRepBuilderAPI_MakeWire()
    for c in comps:
        a, b = max(s_a, c["s0"]), min(s_b, c["s1"])
        if a < b - 1e-6:
            pa, _ = eval_pt(comps, a); pb, _ = eval_pt(comps, b)
            if c["type"] == "line": mk.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(*pa), gp_Pnt(*pb)).Edge())
            else:
                pm, _ = eval_pt(comps, (a+b)/2)
                mk.Add(BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(gp_Pnt(*pa), gp_Pnt(*pm), gp_Pnt(*pb)).Value()).Edge())
    return mk.Wire()

def face_types(shp):
    d = {}; ex = TopExp_Explorer(shp, TopAbs_FACE)
    while ex.More():
        t = BRepAdaptor_Surface(ex.Current()).GetType()
        k = {GeomAbs_Plane: "plane", GeomAbs_Cylinder: "cyl", GeomAbs_BSplineSurface: "bspl"}.get(t, str(t)); d[k] = d.get(k, 0)+1; ex.Next()
    return d

def flip_check(shp, comps, normals, s_list):
    """Distance from the shape to the intended bead-top point (p + n*DEPTH) and to its mirror (p - n*DEPTH).
    No twist: d_top ~ 0.  180-degree flip: d_mirror ~ 0."""
    out = []
    for s in s_list:
        p, _ = eval_pt(comps, s); n = eval_n(comps, normals, s)
        d1 = BRepExtrema_DistShapeShape(shp, BRepBuilderAPI_MakeVertex(gp_Pnt(*vadd(p, n, DEPTH))).Vertex()).Value()
        d2 = BRepExtrema_DistShapeShape(shp, BRepBuilderAPI_MakeVertex(gp_Pnt(*vadd(p, n, -DEPTH))).Vertex()).Value()
        out.append(f"s={s:5.0f}: top {d1:4.1f} / mirror {d2:4.1f}")
    return " | ".join(out)

def gemini_multisection(comps, normals, total, mode="default"):
    s_start, s_end, D = 40.0, total-40.0, 10.0
    s_pts = [0.0, s_start, s_start+D, s_end-D, s_end, total]
    # spine subdivided at s_pts, with the exact middle
    mk = BRepBuilderAPI_MakeWire(); verts = []
    for a, b in zip(s_pts[:-1], s_pts[1:]):
        w = sub_wire(comps, a, b)
        ex = TopExp_Explorer(w, 6)  # TopAbs_EDGE = 6
        while ex.More(): mk.Add(topods.Edge(ex.Current())); ex.Next()
    spine = mk.Wire()
    ps = BRepOffsetAPI_MakePipeShell(spine)
    if mode == "binormal": ps.SetMode(gp_Dir(0, 1, 0))
    if mode == "aux":
        tr = gp_Trsf(); tr.SetTranslation(gp_Vec(0, W, 0))
        ps.SetMode(topods.Wire(BRepBuilderAPI_Transform(spine, tr, True).Shape()), True)
    for i, s in enumerate(s_pts):
        p, t = eval_pt(comps, s); n = eval_n(comps, normals, s)
        prof = profile(DEPTH if i in (2, 3) else 0.0)
        ps.Add(orient(prof, p, t, n), BRepBuilderAPI_MakeVertex(gp_Pnt(*p)).Vertex())
    ps.Build()
    if not ps.IsDone(): return None
    return ps.Shape()

def decomposed(comps, normals, total):
    """Recommended: plate = single-section sweep of the flat line (analytic faces);
    bead = single-section sweep of the bead-only profile along the middle sub-spine."""
    s_start, s_end = 50.0, total-50.0
    p, t = eval_pt(comps, 0.0); n = eval_n(comps, normals, 0.0)
    ps = BRepOffsetAPI_MakePipeShell(spine_wire(comps)); ps.Add(orient(profile(0.0), p, t, n)); ps.Build()
    plate = ps.Shape()
    mid = sub_wire(comps, s_start, s_end)
    p, t = eval_pt(comps, s_start); n = eval_n(comps, normals, s_start)
    pb = BRepOffsetAPI_MakePipeShell(mid); pb.Add(orient(profile(DEPTH, bead_only=True), p, t, n)); pb.Build()
    return plate, pb.Shape()

if __name__ == "__main__":
    from OCC.Core.TopAbs import TopAbs_VERTEX
    from OCC.Core.BRep import BRep_Tool
    def verts(w):
        ex = TopExp_Explorer(w, TopAbs_VERTEX); out = set()
        while ex.More():
            P = BRep_Tool.Pnt(topods.Vertex(ex.Current())); out.add((round(P.X(), 1), round(P.Y(), 1), round(P.Z(), 1))); ex.Next()
        return sorted(out)
    pw = profile(DEPTH, bead_only=True)
    print("placement check, origin (50,0,0), t=(1,0,0), n=(0,0,1); expected top corners (50,+-3,6)")
    print("  buggy  :", verts(orient_buggy(pw, (50, 0, 0), (1, 0, 0), (0, 0, 1))))
    print("  fixed  :", verts(orient(pw, (50, 0, 0), (1, 0, 0), (0, 0, 1))))
    for label, a1, a2, tilt in [("S-chain planar", 70, -70, 0), ("S-chain tilt5", 70, -70, 5), ("C-chain planar", 70, 70, 0), ("C-chain tilt5", 70, 70, 5), ("S-chain tilt5 shallow", 30, -30, 5)]:
        comps, normals, total = chain(a1, a2, tilt)
        s_probe = [60.0, total/2, total-60.0]
        print(f"=== {label}  (spine length {total:.0f})")
        for mode in ("default", "binormal", "aux"):
            shp = gemini_multisection(comps, normals, total, mode)
            if shp is None: print(f"  multi-section {mode:8s}: BUILD FAILED"); continue
            print(f"  multi-section {mode:8s}: valid={BRepCheck_Analyzer(shp).IsValid()} faces={face_types(shp)}  {flip_check(shp, comps, normals, s_probe)}")
        plate, bead = decomposed(comps, normals, total)
        print(f"  decomposed plate: faces={face_types(plate)} | bead: faces={face_types(bead)}  {flip_check(bead, comps, normals, s_probe)}")
