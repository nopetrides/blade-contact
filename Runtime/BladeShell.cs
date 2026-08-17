using UnityEngine;

namespace BladeContact
{
    /// <summary>
    /// Registers authored blade-contact geometry on a dynamic sword. Feature data is supplied by the
    /// consumer; the package contains no specimen geometry and no thesis-specific bind policy.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class BladeShell : MonoBehaviour
    {
        [SerializeField] private Rigidbody body;

        [Tooltip("Authored contact features, expressed in this transform's local space.")]
        [SerializeField] private BladeFeature[] features = new BladeFeature[0];

        private BladeShellData cachedData;

        /// <summary>The dynamic body receiving custom sword-to-sword response.</summary>
        public Rigidbody Body => body;

        /// <summary>Pose-independent snapshot of the authored features, built once and reused.</summary>
        public BladeShellData Data => cachedData ??= new BladeShellData(features);

        /// <summary>Current world pose of this shell's local feature space.</summary>
        public BladePose CurrentPose => new BladePose(transform.position, transform.rotation);

        /// <summary>Discards the cached snapshot after the authored features change.</summary>
        public void InvalidateData() => cachedData = null;

        private void Reset()
        {
            body = GetComponentInParent<Rigidbody>();
        }

        private void OnValidate()
        {
            if (body == null)
                body = GetComponentInParent<Rigidbody>();

            cachedData = null;
        }
    }
}
