<p align="center">
<img src="https://raw.githubusercontent.com/isl-org/Open3D/main/docs/_static/open3d_logo_horizontal.png" width="320" />
</p>

# Open3D: 3D 데이터 처리를 위한 현대적 라이브러리

<h4>
    <a href="https://www.open3d.org">홈페이지</a> |
    <a href="https://www.open3d.org/docs">문서</a> |
    <a href="https://www.open3d.org/docs/release/getting_started.html">빠른 시작</a> |
    <a href="https://www.open3d.org/docs/release/compilation.html">컴파일</a> |
    <a href="https://www.open3d.org/docs/release/index.html#python-api-index">Python</a> |
    <a href="https://www.open3d.org/docs/release/cpp_api.html">C++</a> |
    <a href="https://github.com/isl-org/Open3D-ML">Open3D-ML</a> |
    <a href="https://github.com/isl-org/Open3D/releases">뷰어</a> |
    <a href="https://www.open3d.org/docs/release/contribute/contribute.html">기여</a> |
    <a href="https://www.youtube.com/channel/UCRJBlASPfPBtPXJSPffJV-w">데모</a> |
    <a href="https://github.com/isl-org/Open3D/discussions">포럼</a>
</h4>

Open3D는 3D 데이터를 다루는 소프트웨어의 빠른 개발을 지원하는 오픈 소스 라이브러리입니다. Open3D 프론트엔드는 C++와 Python 모두에서 엄선된 데이터 구조와 알고리즘 세트를 제공합니다. 백엔드는 높은 수준으로 최적화되어 있으며 병렬 처리를 위해 구성되어 있습니다. 오픈 소스 커뮤니티의 기여를 환영합니다.

[![Ubuntu CI](https://github.com/isl-org/Open3D/actions/workflows/ubuntu.yml/badge.svg)](https://github.com/isl-org/Open3D/actions?query=workflow%3A%22Ubuntu+CI%22)
[![macOS CI](https://github.com/isl-org/Open3D/actions/workflows/macos.yml/badge.svg)](https://github.com/isl-org/Open3D/actions?query=workflow%3A%22macOS+CI%22)
[![Windows CI](https://github.com/isl-org/Open3D/actions/workflows/windows.yml/badge.svg)](https://github.com/isl-org/Open3D/actions?query=workflow%3A%22Windows+CI%22)

**Open3D의 핵심 기능:**

-   3D 데이터 구조
-   3D 데이터 처리 알고리즘
-   장면 재구성
-   표면 정합
-   3D 시각화
-   물리 기반 렌더링(PBR)
-   PyTorch 및 TensorFlow를 활용한 3D 머신러닝 지원
-   핵심 3D 연산의 GPU 가속
-   C++ 및 Python 지원

다음은 Open3D의 각 구성 요소가 어떻게 맞물려 전체 end-to-end 파이프라인을 구성하는지에 대한 간략한 개요입니다.

![Open3D_layers](https://github.com/isl-org/Open3D/assets/41028320/e9b8645a-a823-4d78-8310-e85207bbc3e4)

자세한 내용은 [Open3D 문서](https://www.open3d.org/docs)를 참고하세요.

Open3D를 소개하는 현대 3D 데이터 처리 입문서도 확인해 보세요.

<img src="https://learning.oreilly.com/covers/urn:orm:book:9781098161323/400w/" width="240" alt="3D Data Science with Python" />

[3D Data Science with Python](https://learning.oreilly.com/library/view/3d-data-science/9781098161323/)
by [Dr. Florent Poux](https://www.graphics.rwth-aachen.de/person/306/)

저자의 말:

> 이 책 전반에서 Open3D가 실용적인 예제와 코드 샘플을 통해 효율적인 포인트 클라우드 처리, 메시 조작, 3D 시각화를 어떻게 가능하게 하는지 보여줍니다. 독자는 실제 3D 데이터 과학 워크플로에서 정합(registration), 세그멘테이션, 특징 추출을 위해 Open3D의 강력한 기능을 활용하는 방법을 학습합니다.

## Python 빠른 시작

사전 빌드된 pip 패키지는 Ubuntu 20.04+, macOS 10.15+, Windows 10+ (64비트)에서 Python 3.10–3.14를 지원합니다.

```bash
# 설치
pip install open3d       # 또는
pip install open3d-cpu   # x86_64 Linux에서 더 작은 CPU 전용 wheel (v0.17+)

# 설치 확인
python -c "import open3d as o3d; print(o3d.__version__)"

# Python API
python -c "import open3d as o3d; \
           mesh = o3d.geometry.TriangleMesh.create_sphere(); \
           mesh.compute_vertex_normals(); \
           o3d.visualization.draw(mesh, raw_mode=True)"

# Open3D CLI
open3d example visualization/draw
```

Open3D의 최신 기능을 사용하려면 [개발용 pip 패키지](https://www.open3d.org/docs/latest/getting_started.html#development-version-pip)를 설치하세요. 소스에서 Open3D를 컴파일하려면 [소스 컴파일](https://www.open3d.org/docs/release/compilation.html) 문서를 참고하세요.

## C++ 빠른 시작

Open3D C++ API를 시작하려면 다음 링크를 확인하세요.

-   Open3D 바이너리 패키지 다운로드: [릴리스](https://github.com/isl-org/Open3D/releases) 또는 [최신 개발 버전](https://www.open3d.org/docs/latest/getting_started.html#c)
-   [소스에서 Open3D 컴파일](https://www.open3d.org/docs/release/compilation.html)
-   [Open3D C++ API](https://www.open3d.org/docs/release/cpp_api.html)

C++ 프로젝트에서 Open3D를 사용하는 예제는 다음을 참고하세요.

-   [CMake에서 사전 설치된 Open3D 패키지 찾기](https://github.com/isl-org/open3d-cmake-find-package)
-   [CMake 외부 프로젝트로 Open3D 사용](https://github.com/isl-org/open3d-cmake-external-project)

## Open3D-Viewer 앱

<img width="480" src="https://raw.githubusercontent.com/isl-org/Open3D/main/docs/_static/open3d_viewer.png">

Open3D-Viewer는 Debian(Ubuntu), macOS, Windows에서 사용할 수 있는 독립 실행형 3D 뷰어 앱입니다. [릴리스 페이지](https://github.com/isl-org/Open3D/releases)에서 Open3D Viewer를 다운로드할 수 있습니다.

## Open3D-ML

<img width="480" src="https://raw.githubusercontent.com/isl-org/Open3D-ML/main/docs/images/getting_started_ml_visualizer.gif">

Open3D-ML은 3D 머신러닝 작업을 위한 Open3D 확장입니다. Open3D 코어 라이브러리 위에 구축되어 3D 데이터 처리를 위한 머신러닝 도구를 확장합니다. 사용해 보려면 PyTorch 또는 TensorFlow와 함께 Open3D를 설치하고 [Open3D-ML](https://github.com/isl-org/Open3D-ML)을 확인하세요.

## 소통 채널

-   [GitHub Issue](https://github.com/isl-org/Open3D/issues): 버그 보고, 기능 요청 등
-   [포럼](https://github.com/isl-org/Open3D/discussions): Open3D 사용 관련 논의
-   [Discord 채팅](https://discord.gg/D35BGvn): 온라인 채팅, 논의, 다른 사용자 및 개발자와의 협업

## 인용

Open3D를 사용하는 경우 [논문](https://arxiv.org/abs/1801.09847)을 인용해 주세요.

```bib
@article{Zhou2018,
    author    = {Qian-Yi Zhou and Jaesik Park and Vladlen Koltun},
    title     = {{Open3D}: {A} Modern Library for {3D} Data Processing},
    journal   = {arXiv:1801.09847},
    year      = {2018},
}
```
