using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Threading;

namespace RecastSharp
{
    /// <summary>
    /// Resolves the native RecastWrapper library next to the managed assembly,
    /// regardless of which subfolder the build dropped it in. Registered once at
    /// module load so every <c>[DllImport(NativeLibraryResolver.LibraryName)]</c>
    /// in this assembly resolves correctly on .NET 8+.
    /// </summary>
    internal static class NativeLibraryResolver
    {
        /// <summary>Logical name used by every [DllImport] in this assembly.</summary>
        internal const string LibraryName = "RecastWrapper64";

        private static int _registered;

        [System.Diagnostics.CodeAnalysis.SuppressMessage(
            "Usage",
            "CA2255:The 'ModuleInitializer' attribute should not be used in libraries",
            Justification = "The DllImport resolver must be registered before the first P/Invoke; a module initializer is the correct mechanism to guarantee that.")]
        [ModuleInitializer]
        internal static void Register()
        {
            if (Interlocked.Exchange(ref _registered, 1) == 1)
            {
                return;
            }

            NativeLibrary.SetDllImportResolver(typeof(NativeLibraryResolver).Assembly, Resolve);
        }

        private static IntPtr Resolve(string libraryName, Assembly assembly, DllImportSearchPath? searchPath)
        {
            if (libraryName != LibraryName)
            {
                return IntPtr.Zero;
            }

            bool windows = OperatingSystem.IsWindows();
            string fileName = windows ? "RecastWrapper64.dll" : "libRecastWrapper64.so";
            string rid = windows ? "win-x64" : "linux-x64";

            foreach (string dir in ProbeDirectories(rid))
            {
                string candidate = Path.Combine(dir, fileName);
                if (File.Exists(candidate) && NativeLibrary.TryLoad(candidate, out IntPtr handle))
                {
                    return handle;
                }
            }

            // Nothing matched; fall back to the default OS search.
            return IntPtr.Zero;
        }

        private static IEnumerable<string> ProbeDirectories(string rid)
        {
            string baseDir = AppContext.BaseDirectory;
            yield return baseDir;
            yield return Path.Combine(baseDir, "Costura64");
            yield return Path.Combine(baseDir, "native");
            yield return Path.Combine(baseDir, "runtimes", rid, "native");
        }
    }
}
