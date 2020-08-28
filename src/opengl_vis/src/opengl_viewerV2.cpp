//
// Created by kevin on 2020/4/26.
//


#include <opengl_viewerV2.h>


namespace openglViewer{

OpenGLViewerV2::OpenGLViewerV2(){

    window_width = Config::get<float>("window_width");
    window_height = Config::get<float>("window_height");

    vsprofile = Config::get<string> ( "vsprofile" );
    fsprofile = Config::get<string> ( "fsprofile" );

    string infoAdd = Config::get<string> ( "info_dir" );

    // read PointCloud and Mesh Data
    cout << "info_dir: "<< infoAdd << endl;

    ifstream info_fin ( infoAdd );
    if ( !info_fin)
    {
        cout<<"could not open pointcloud file!"<<endl;
        return;
    }

    while ( !info_fin.eof() )
    {
        //read point number
        char *ifBuffer = new char[256]();        // remember to delete!
        info_fin.getline(ifBuffer, 256);
        TextureV2::Ptr tv2p = TextureV2::createTexture(ifBuffer, 256);
        textures.push_back(tv2p);
        delete []ifBuffer;
        if ( info_fin.good() == false )
            break;
    }
    info_fin.close();

    pos_normalize();

    texture_num = textures.size();

}

OpenGLViewerV2::~OpenGLViewerV2(){
    //delete []vertices;
    //delete []indices;
}

void OpenGLViewerV2::pos_normalize()
{
    float minX, minY, minZ;
    float maxX, maxY, maxZ;
    float scale;
    float origX, origY, origZ;

    minX = minY = minZ = 10000.0;
    maxX = maxY = maxZ = -10000.0;

    for(TextureV2::Ptr txv2 : textures) {
        for (int i = 0; i < txv2->point_num; i++) {
            if (txv2->vertices[i * 5] > maxX) {
                maxX = txv2->vertices[i * 5];
            }
            if (txv2->vertices[i * 5 + 1] > maxY) {
                maxY = txv2->vertices[i * 5 + 1];
            }
            if (txv2->vertices[i * 5 + 2] > maxZ) {
                maxZ = txv2->vertices[i * 5 + 2];
            }

            if (txv2->vertices[i * 5] < minX) {
                minX = txv2->vertices[i * 5];
            }
            if (txv2->vertices[i * 5 + 1] < minY) {
                minY = txv2->vertices[i * 5 + 1];
            }
            if (txv2->vertices[i * 5 + 2] < minZ) {
                minZ = txv2->vertices[i * 5 + 2];
            }
        }
    }

    origX = (maxX + minX)/2;
    origY = (maxY + minY)/2;
    origZ = (maxZ + minZ)/2;

    scale = min(2/(maxX - minX), min(2/(maxY - minY), 2/(maxZ - minZ)));

    cout << "------------------normalize process--------------------" << endl;
    cout << "max XYZ: " << maxX << " " << maxY << " " <<maxZ << endl;
    cout << "min XYZ: " << minX << " " << minY << " " << minZ << endl;
    cout << "scale: " << scale << endl;
    cout << "origXYZ:" << origX << " " << origY << " " << origZ << endl;

    for(TextureV2::Ptr txv2 : textures) {
        for (int i = 0; i < txv2->point_num; i++) {
            txv2->vertices[i * 5] -= origX;
            txv2->vertices[i * 5 + 1] -= origY;
            txv2->vertices[i * 5 + 2] -= origZ;
            txv2->vertices[i * 5] *= scale;
            txv2->vertices[i * 5 + 1] *= scale;
            txv2->vertices[i * 5 + 2] *= scale;
        }
    }

}

//---------------------------Texture Part-----------------------------------------

TextureV2::TextureV2()
:id_(0), width_(0), height_(0), nrChannels_(0), fileDir_(nullptr), data_(nullptr)
{}
TextureV2::TextureV2(unsigned int id, const char* info, int num)
:id_(id)
{
    unsigned int frameId, frameW, frameH;
    char pointFile[96] = {0};
    char meshFile[96] = {0};
    char imgFile[96] = {0};
    sscanf(info,"%d %s %d %d %s %d %s %d", &frameId, imgFile, &frameW, &frameH, pointFile, &point_num, meshFile, &tri_num);
    cout << "number of points: " << point_num << endl;

    vertices = new float[point_num*5];
    indices = new unsigned int[tri_num*3];

    stbi_set_flip_vertically_on_load(true);
    data_ = stbi_load(imgFile, &width_, &height_, &nrChannels_, 0);

    ifstream point_fin(pointFile);
    if ( !point_fin)
    {
        cout<<"could not open point file!"<<endl;
        return;
    }

    for(int i=0; i<point_num; i++)
    {
        double data[6];
        for ( double& d:data ) point_fin>>d;
        vertices[i*5] = data[1];
        vertices[i*5+1] = data[2];
        vertices[i*5+2] = data[3];
        vertices[i*5+3] = data[4];
        vertices[i*5+4] = data[5];

        if ( !point_fin.good() )
            break;
    }
    point_fin.close();

    ifstream mesh_fin(meshFile);
    if ( !mesh_fin)
    {
        cout<<"could not open mesh file!"<<endl;
        return;
    }

    for(int i=0; i<tri_num; i++)
    {
        int idata[4];
        for ( int& d:idata ) mesh_fin>>d;
        indices[i*3] = idata[1];
        indices[i*3+1] = idata[2];
        indices[i*3+2] = idata[3];

        if ( !mesh_fin.good() )
            break;
    }
    mesh_fin.close();

    normalize_texture();

}

TextureV2::~TextureV2(){
    stbi_image_free(data_);
}

TextureV2::Ptr TextureV2::createTexture(const char* info, int num){
    return TextureV2::Ptr(new TextureV2(factory_id_++, info, num));
}

//进行标准化
void TextureV2::normalize_texture()
{
    for (int i = 0; i < point_num; i++) {
        // pixel_coord normalize
        vertices[i*5+4] = height_ - vertices[i*5+4];
        vertices[i*5+3] /= width_;
        vertices[i*5+4] /= height_;
    }
}



unsigned int TextureV2::factory_id_ = 0;

}

