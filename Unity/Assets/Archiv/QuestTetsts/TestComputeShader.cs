using UnityEngine;

[RequireComponent(typeof(Renderer))]
public class ComputeTexture : MonoBehaviour
{
    public ComputeShader computeShader;
    public int textureSize = 256;

    private RenderTexture renderTexture;

    void Start()
    {
        // 1. RenderTexture erstellen
        renderTexture = new RenderTexture(textureSize, textureSize, 0);
        renderTexture.enableRandomWrite = true;
        renderTexture.graphicsFormat = UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_UNorm;
        renderTexture.Create();

        // 2. Compute Shader ausführen
        int kernel = computeShader.FindKernel("CSMain");
        computeShader.SetTexture(kernel, "Result", renderTexture);
        computeShader.Dispatch(kernel, textureSize / 8, textureSize / 8, 1);

        // 3. Material vorbereiten
        var renderer = GetComponent<Renderer>();
        var mat = renderer.material;
        mat.shader = Shader.Find("Unlit/Texture");
        mat.mainTexture = renderTexture;
    }

    void OnDestroy()
    {
        if (renderTexture != null)
        {
            renderTexture.Release();
        }
    }
}
