Shader "Custom/PointCloudShader"
{
    Properties {}
    SubShader
    {
        Tags { "RenderType" = "Opaque" }
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 3.0

            #include "UnityCG.cginc"

            StructuredBuffer<float3> vertexBuffer;
            StructuredBuffer<float4> colorBuffer;

            struct appdata { uint vertexID : SV_VertexID; };
            struct v2f {
                float4 pos : SV_POSITION;
                float4 color : COLOR;
                float pointSize : PSIZE;   // <-- Punktgröße hinzufügen
            };

            v2f vert(appdata v)
            {
                v2f o;
                float3 pos = vertexBuffer[v.vertexID];
                o.pos = UnityObjectToClipPos(float4(pos, 1.0));
                o.color = colorBuffer[v.vertexID];
                o.pointSize = 5.0;  // z.B. 5 Pixel groß
                return o;
            }

            float4 frag(v2f i) : SV_Target { return i.color; }
            ENDCG
        }
    }
    FallBack "Diffuse"
}
