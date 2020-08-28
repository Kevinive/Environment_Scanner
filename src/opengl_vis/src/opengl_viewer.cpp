//
// Created by kevin on 2020/4/26.
//


#include <opengl_viewer.h>

//pre set: env_rebuilder::Config::setParameterFile ( PARAM_FILE );

namespace openglViewer{

OpenGLViewer::OpenGLViewer(){

    window_width = Config::get<float>("window_width");
    window_height = Config::get<float>("window_height");

    vsprofile = Config::get<string> ( "vsprofile" );
    fsprofile = Config::get<string> ( "fsprofile" );

    string pcAdd = Config::get<string> ( "pointcloud_dir" );
    string meshAdd = Config::get<string> ( "mesh_dir" );

    // read PointCloud and Mesh Data
    cout << "pointcloud_dir: "<< pcAdd << endl;
    cout << "mesh_dir: "<< meshAdd << endl;

    ifstream point_fin ( pcAdd );
    if ( !point_fin)
    {
        cout<<"could not open pointcloud file!"<<endl;
        return;
    }

    //read point number
    char *ifBuffer = new char[64]();        // remember to delete!
    point_fin.getline(ifBuffer, 64);
    sscanf(ifBuffer,"%d",&point_num);
    cout << "number of points: " << point_num << endl;

    vertices = new float[point_num*3];

    for(int i=0; i<point_num; i++)
    {
        double data[7];
        for ( double& d:data ) point_fin>>d;
        vertices[i*3] = data[1];
        vertices[i*3+1] = data[2];
        vertices[i*3+2] = data[3];

        if ( !point_fin.good() )
            break;
    }
    point_fin.close();
    delete []ifBuffer;

    // read mesh file
    ifstream mesh_fin ( meshAdd );
    if ( !mesh_fin )
    {
        cout<<"could not open mesh file!"<<endl;
        return;
    }

    //read mesh number
    ifBuffer = new char[64]();        // remember to delete!
    mesh_fin.getline(ifBuffer, 64);
    sscanf(ifBuffer,"%d",&tri_num);
    cout << "number of points: " << tri_num << endl;

    indices = new unsigned int[tri_num*3];

    for(int i=0; i<tri_num; i++)
    {
        int idata[4];
        double ddata[6];
        string textureAdd;
        for ( int& d:idata ) mesh_fin>>d;
        for ( double& d:ddata ) mesh_fin>>d;
        mesh_fin>>textureAdd;
        indices[i*3] = idata[1];
        indices[i*3+1] = idata[2];
        indices[i*3+2] = idata[3];


        if ( !mesh_fin.good() )
            break;
    }
    mesh_fin.close();
    delete []ifBuffer;

}

OpenGLViewer::~OpenGLViewer(){
    //delete []vertices;
    //delete []indices;
}

//---------------------------Texture Part-----------------------------------------

Texture::Texture()
:id_(0), width_(0), height_(0), nrChannels_(0), fileDir_(nullptr), data_(nullptr)
{}
Texture::Texture(unsigned int id, string dir)
:id_(id), fileDir_(dir)
{
    stbi_set_flip_vertically_on_load(true);
    data_ = stbi_load(dir.c_str(), &width_, &height_, &nrChannels_, 0);
}

Texture::~Texture(){
    stbi_image_free(data_);
}

Texture::Ptr Texture::createTexture(string dir){
    return Texture::Ptr(new Texture(factory_id_++, dir));
}

unsigned int Texture::factory_id_ = 0;

}


