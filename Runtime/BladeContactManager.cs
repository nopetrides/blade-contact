using UnityEngine;

namespace BladeContact
{
    /// <summary>
    /// Future owner of registered BladeShell-to-BladeShell contact. This scaffold intentionally
    /// provides no kinematic pose authority: registered swords remain dynamic Rigidbodies.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class BladeContactManager : MonoBehaviour
    {
    }
}
