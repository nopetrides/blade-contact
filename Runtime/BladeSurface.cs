using System;
using UnityEngine;

namespace BladeContact
{
    /// <summary>
    /// One authored surface piece of a blade shell: a triangle carrying a semantic identity.
    /// </summary>
    /// <remarks>
    /// Surfaces are stored as triangles rather than quads or polygons because a triangle is always
    /// planar and always convex. A quad spanning two cross-section stations of a tapering blade is
    /// neither, and the distance queries would silently lose accuracy on exactly the thin, angled
    /// geometry this system exists to handle.
    ///
    /// A surface has real area and no inflation radius. Blade thickness comes from the authored
    /// positions of the opposing faces, which converge to zero at a designated sharp edge; it is never
    /// produced by sweeping a radius around a centre line.
    /// </remarks>
    [Serializable]
    public struct BladeSurface
    {
        [SerializeField] private string id;
        [SerializeField] private BladeFeatureType type;
        [SerializeField] private Vector3 localA;
        [SerializeField] private Vector3 localB;
        [SerializeField] private Vector3 localC;

        public BladeSurface(string id, BladeFeatureType type, Vector3 localA, Vector3 localB, Vector3 localC)
        {
            this.id = id;
            this.type = type;
            this.localA = localA;
            this.localB = localB;
            this.localC = localC;
        }

        public string Id => id;

        /// <summary>Authored semantic identity. Never inferred from the geometry.</summary>
        public BladeFeatureType Type => type;

        public Vector3 LocalA => localA;
        public Vector3 LocalB => localB;
        public Vector3 LocalC => localC;

        public Vector3 LocalCentre => (localA + localB + localC) / 3f;

        /// <summary>Radius of a bounding sphere about <see cref="LocalCentre"/>, for broadphase rejection.</summary>
        public float LocalBoundingRadius
        {
            get
            {
                Vector3 centre = LocalCentre;
                return Mathf.Sqrt(Mathf.Max(
                    (localA - centre).sqrMagnitude,
                    Mathf.Max((localB - centre).sqrMagnitude, (localC - centre).sqrMagnitude)));
            }
        }

        /// <summary>Largest distance from the shell origin reached by this surface.</summary>
        public float LocalExtent => Mathf.Sqrt(Mathf.Max(
            localA.sqrMagnitude, Mathf.Max(localB.sqrMagnitude, localC.sqrMagnitude)));
    }
}
