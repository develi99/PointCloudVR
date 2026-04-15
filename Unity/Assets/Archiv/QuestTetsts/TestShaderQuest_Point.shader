Shader "Custom/TestPointShader"
{
    Properties
    {
        _PointSize("Point Size", Float) = 10.0
    }

    SubShader
    {
        Tags { "RenderType"="Opaque" }
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 3.0
            #include "UnityCG.cginc"

            StructuredBuffer<float3> vertexBuffer;
            StructuredBuffer<float4> colorBuffer;
            float _PointSize;

            struct appdata
            {
                uint vertexID : SV_VertexID;
            };

            struct v2f
            {
                float4 pos : SV_POSITION;
                float4 color : COLOR;
                float psize : PSIZE;
            };

            v2f vert(appdata v)
            {
                v2f o;
                float3 worldPos = vertexBuffer[v.vertexID];
                o.pos = UnityObjectToClipPos(float4(worldPos, 1.0));
                o.color = colorBuffer[v.vertexID];
                o.psize = _PointSize;
                return o;
            }

            float4 frag(v2f i) : SV_Target
            {
                return i.color;
            }
            ENDCG
        }
    }
}

