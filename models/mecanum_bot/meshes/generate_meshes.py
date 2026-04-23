"""
generate_meshes.py  —  Realistic mecanum wheel meshes for Ignition Fortress
=============================================================================

Design parameters (all must be consistent):
  wheel_radius     = 0.100 m   (physics sphere collision radius)
  roller_center_r  = 0.075 m   (distance from wheel axis to roller axis)
  roller_max_r     = 0.024 m   (barrel half-diameter at widest point)
  roller_tip_r     = 0.075 + 0.024 = 0.099 m  ← just inside flange edge
  flange_outer_r   = 0.100 m   (= wheel radius, covers roller tips)
  N_rollers        = 9         (spacing check: 2*0.075*sin(π/9) = 0.051 m > 0.048 m diam ✓)

Outputs:
  wheel_frame.stl      — aluminium hub + two disc flanges (silver in SDF)
  mecanum_roller.stl   — single barrel roller, axis along Z (rubber in SDF)
"""

import numpy as np, struct, os

OUT = os.path.dirname(os.path.abspath(__file__))

# ── STL writer ───────────────────────────────────────────────────────────────
def write_stl(path, tris):
    with open(path, 'wb') as f:
        f.write(b'mecanum_mesh' + b'\0'*68)
        f.write(struct.pack('<I', len(tris)))
        for tri in tris:
            v = [np.asarray(p, np.float32) for p in tri]
            n = np.cross(v[1]-v[0], v[2]-v[0]).astype(np.float32)
            nl = np.linalg.norm(n)
            if nl > 1e-10: n /= nl
            f.write(struct.pack('<3f', *n))
            for vv in v: f.write(struct.pack('<3f', *vv))
            f.write(b'\x00\x00')
    print(f"  {len(tris):5d} triangles → {os.path.basename(path)}")

# ── Primitives ───────────────────────────────────────────────────────────────
def cyl_side(r, z0, z1, n=60, outward=True):
    T = []
    for j in range(n):
        a0, a1 = 2*np.pi*j/n, 2*np.pi*(j+1)/n
        p00=(r*np.cos(a0),r*np.sin(a0),z0); p01=(r*np.cos(a1),r*np.sin(a1),z0)
        p10=(r*np.cos(a0),r*np.sin(a0),z1); p11=(r*np.cos(a1),r*np.sin(a1),z1)
        if outward: T+=[(p00,p10,p11),(p00,p11,p01)]
        else:       T+=[(p00,p11,p10),(p00,p01,p11)]
    return T

def annulus(r_in, r_out, z, up=True, n=60):
    T = []
    for j in range(n):
        a0, a1 = 2*np.pi*j/n, 2*np.pi*(j+1)/n
        pi0=(r_in*np.cos(a0),r_in*np.sin(a0),z);  pi1=(r_in*np.cos(a1),r_in*np.sin(a1),z)
        po0=(r_out*np.cos(a0),r_out*np.sin(a0),z); po1=(r_out*np.cos(a1),r_out*np.sin(a1),z)
        if up: T+=[(pi0,po0,po1),(pi0,po1,pi1)]
        else:  T+=[(pi0,po1,po0),(pi0,pi1,po1)]
    return T

def tube(r_in, r_out, z0, z1, n=60):
    """Closed hollow cylinder section."""
    return (cyl_side(r_out, z0, z1, n, outward=True) +
            cyl_side(r_in,  z0, z1, n, outward=False) +
            annulus(r_in, r_out, z0, up=False, n=n) +
            annulus(r_in, r_out, z1, up=True,  n=n))

def solid_cyl(r, z0, z1, n=48):
    T = cyl_side(r, z0, z1, n)
    for j in range(n):
        a0,a1=2*np.pi*j/n,2*np.pi*(j+1)/n
        T.append(((0,0,z0),(r*np.cos(a1),r*np.sin(a1),z0),(r*np.cos(a0),r*np.sin(a0),z0)))
        T.append(((0,0,z1),(r*np.cos(a0),r*np.sin(a0),z1),(r*np.cos(a1),r*np.sin(a1),z1)))
    return T

# ── Chamfered ring (bevelled outer edge of flange) ───────────────────────────
def chamfered_disc(r_in, r_out, z0, z1, chamfer=0.003, n=60):
    """Flat annulus with a small 45° chamfer on the outer edge."""
    T = []
    r_ch = r_out - chamfer
    z_mid_lo = z0 + chamfer
    z_mid_hi = z1 - chamfer
    # Inner bore cylinder
    T += cyl_side(r_in, z0, z1, n, outward=False)
    # Bottom face (annulus from r_in to r_ch + chamfer triangle)
    T += annulus(r_in, r_ch, z0, up=False, n=n)
    # Bottom chamfer: connects (r_ch, z0) to (r_out, z_mid_lo)
    for j in range(n):
        a0,a1=2*np.pi*j/n,2*np.pi*(j+1)/n
        p0=(r_ch *np.cos(a0),r_ch *np.sin(a0),z0)
        p1=(r_ch *np.cos(a1),r_ch *np.sin(a1),z0)
        p2=(r_out*np.cos(a0),r_out*np.sin(a0),z_mid_lo)
        p3=(r_out*np.cos(a1),r_out*np.sin(a1),z_mid_lo)
        T+=[(p0,p2,p3),(p0,p3,p1)]
    # Side wall (outer) from z_mid_lo to z_mid_hi
    if z_mid_hi > z_mid_lo:
        T += cyl_side(r_out, z_mid_lo, z_mid_hi, n, outward=True)
    # Top chamfer
    for j in range(n):
        a0,a1=2*np.pi*j/n,2*np.pi*(j+1)/n
        p0=(r_out*np.cos(a0),r_out*np.sin(a0),z_mid_hi)
        p1=(r_out*np.cos(a1),r_out*np.sin(a1),z_mid_hi)
        p2=(r_ch *np.cos(a0),r_ch *np.sin(a0),z1)
        p3=(r_ch *np.cos(a1),r_ch *np.sin(a1),z1)
        T+=[(p0,p2,p3),(p0,p3,p1)]
    # Top face
    T += annulus(r_in, r_ch, z1, up=True, n=n)
    return T

# ── Barrel roller ─────────────────────────────────────────────────────────────
def make_roller(max_r=0.024, min_r=0.008, length=0.054, nc=40, nl=40):
    """
    Barrel roller centred at origin, axis along Z.
    Profile: r(t) = min_r + (max_r-min_r)*cos(t*π/2)^2,  t in [-1,1]
    Very thin at ends (min_r=0.008) → dramatic barrel shape visible from all angles.
    """
    T = []
    half = length/2
    zs = np.linspace(-half, half, nl+1)

    def r(z):
        t = z/half
        return min_r + (max_r-min_r)*np.cos(t*np.pi/2)**2

    # Side surface
    for i in range(nl):
        z0,z1 = zs[i],zs[i+1]
        r0,r1 = r(z0),r(z1)
        for j in range(nc):
            a0,a1=2*np.pi*j/nc,2*np.pi*(j+1)/nc
            p00=(r0*np.cos(a0),r0*np.sin(a0),z0); p01=(r0*np.cos(a1),r0*np.sin(a1),z0)
            p10=(r1*np.cos(a0),r1*np.sin(a0),z1); p11=(r1*np.cos(a1),r1*np.sin(a1),z1)
            T+=[(p00,p10,p11),(p00,p11,p01)]

    # End caps
    for j in range(nc):
        a0,a1=2*np.pi*j/nc,2*np.pi*(j+1)/nc
        rb,rt=r(zs[0]),r(zs[-1]); zb,zt=zs[0],zs[-1]
        T.append(((0,0,zb),(rb*np.cos(a1),rb*np.sin(a1),zb),(rb*np.cos(a0),rb*np.sin(a0),zb)))
        T.append(((0,0,zt),(rt*np.cos(a0),rt*np.sin(a0),zt),(rt*np.cos(a1),rt*np.sin(a1),zt)))

    return T

# ── Wheel frame ───────────────────────────────────────────────────────────────
def make_frame(
    hub_r     = 0.022,   # axle bore radius
    web_r     = 0.040,   # inner machined web outer radius
    flange_r  = 0.100,   # outer flange radius = wheel contact radius
    gap_half  = 0.028,   # half-gap between flanges (roller zone)
    flange_t  = 0.008,   # flange thickness
    chamfer   = 0.003,   # outer-edge bevel
    n         = 72,      # segments
):
    """
    Structure:
      [rear flange]──[hub cylinder]──[front flange]
       ←flange_t→  ←── gap_half*2 ──→  ←flange_t→

    Inner web (hub_r→web_r) bridges the full axial length for rigidity.
    """
    T = []
    zf0 =  gap_half              # inner face of front flange
    zf1 =  gap_half + flange_t  # outer face of front flange
    zr0 = -gap_half - flange_t  # outer face of rear flange
    zr1 = -gap_half              # inner face of rear flange

    # ── Front flange (chamfered disc)
    T += chamfered_disc(hub_r, flange_r, zf0, zf1, chamfer, n)

    # ── Rear flange (chamfered disc)
    T += chamfered_disc(hub_r, flange_r, zr0, zr1, chamfer, n)

    # ── Hub cylinder between flanges
    T += solid_cyl(hub_r, zr1, zf0, n//2)

    # ── Inner web: thickened hub area (hub_r → web_r) across full axle length
    # This creates the "machined hub" look from the side
    T += tube(hub_r, web_r, zr0, zf1, n//2)

    # ── Axle stub detail: a short protruding cylinder on each side
    stub_r = 0.015
    stub_l = 0.010
    T += solid_cyl(stub_r, zf1,        zf1+stub_l, n//3)
    T += solid_cyl(stub_r, zr0-stub_l, zr0,        n//3)

    return T

# ── Roller pose calculator ────────────────────────────────────────────────────
def rot_matrix(axis, angle):
    axis = np.asarray(axis, float); axis /= np.linalg.norm(axis)
    c,s=np.cos(angle),np.sin(angle); t=1-c; x,y,z=axis
    return np.array([[t*x*x+c,t*x*y-s*z,t*x*z+s*y],
                     [t*x*y+s*z,t*y*y+c,t*y*z-s*x],
                     [t*x*z-s*y,t*y*z+s*x,t*z*z+c]])

def mat_rpy(R):
    sy=np.sqrt(R[0,0]**2+R[1,0]**2)
    if sy>1e-6:
        return np.arctan2(R[2,1],R[2,2]), np.arctan2(-R[2,0],sy), np.arctan2(R[1,0],R[0,0])
    return np.arctan2(-R[1,2],R[1,1]), np.arctan2(-R[2,0],sy), 0.0

def roller_pose_str(theta, rc=0.075, left=True):
    px,py = rc*np.cos(theta), rc*np.sin(theta)
    tang = np.array([-np.sin(theta), np.cos(theta), 0.0])
    ra = (tang + np.array([0,0,1])) if left else (tang - np.array([0,0,1]))
    ra /= np.linalg.norm(ra)
    d = ra[2]
    if abs(d-1)<1e-6: R3=np.eye(3)
    elif abs(d+1)<1e-6: R3=np.diag([1,-1,-1])
    else:
        rax=np.array([-ra[1],ra[0],0]); rax/=np.linalg.norm(rax)
        R3=rot_matrix(rax, np.arccos(d))
    r,p,y = mat_rpy(R3)
    return f"{px:.5f} {py:.5f} 0.0 {r:.5f} {p:.5f} {y:.5f}"

# ── Generate ──────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print("Generating realistic mecanum wheel meshes...")

    write_stl(os.path.join(OUT,'mecanum_roller.stl'), make_roller())
    write_stl(os.path.join(OUT,'wheel_frame.stl'),    make_frame())

    print("\nRoller poses for model.sdf (N=9, rc=0.075 m):")
    for wtype, left in [('FL/RR (left=True)', True), ('FR/RL (left=False)', False)]:
        print(f"  # {wtype}")
        for i in range(9):
            th = 2*np.pi*i/9
            print(f"  <pose>{roller_pose_str(th, left=left)}</pose>")

    print("\nDone.")
