// Generated for ALVisionRecognition version 1.14

#ifndef ALVISIONRECOGNITIONPROXYPOSTHANDLER_H_
#define ALVISIONRECOGNITIONPROXYPOSTHANDLER_H_
#include <boost/shared_ptr.hpp>
#include <alvalue/alvalue.h>
#include <string>
#include <vector>

namespace AL
{
class ALProxy;
class ALVisionRecognitionProxy;

/// <summary>
/// ALVisionRecognition is a module which detects and recognizes learned pictures, like pages of a comic books, faces of objects or even locations.
    ///
    /// The learning stage is done using the Choregraphe interface. Follow the steps in the green doc that will explain how to create your own database.
    ///
    /// The output value is written in ALMemory in the PictureDetected variable.
    ///
    /// It contains an array of tags, with the following format: 
    ///
    ///  
    ///
    /// [ [ TimeStampField ] [ Picture_info_0 , Picture _info_1, . . . , Picture_info_N-1 ] ] 
    ///
    ///  
    ///
    /// with as many Picture_info tags as things currently recognized. 
    ///
    /// Picture_info = [[labels_list], matched_keypoints, ratio, [boundary_points]] 
    ///
    /// with labels_list = [label_0, label_1, ..., label_N-1] and label_n belongs to label_n+1 
    ///
    /// and boundary_points = [[x0,y0], [x1,y1], ..., [xN,yN]] 
    ///
    ///  
    ///
    /// - Labels are the names given to the picture (e.g. \"cover/my book\", or \"fridge corner/kitchen/my flat\"). 
    ///
    /// - matched_keypoints corresponds to the number of keypoints retrieved in the current frame. 
    ///
    /// - ratio represents the number of keypoints found for the object in the current frame divided by the number of keypoints found during the learning stage. 
    ///
    /// - boundary_points is a list of points coordinates in angle values representing the reprojection in the current image of the boundaries selected during the learning stage.
/// ALVisionRecognitionPostHandler gives acces to all bound methods of the module, using a thread around each call.
/// Long running tasks can be stopped by calling stop(id) with the id returned by the post.
/// </summary>
/// \ingroup ALProxies
class ALVisionRecognitionProxyPostHandler
{
  friend class ALVisionRecognitionProxy;

  protected:
  boost::shared_ptr<ALProxy> fParent;

  private:
  void setParent(boost::shared_ptr<ALProxy> pParent);

  public:
  /// <summary>
  /// ALVisionRecognition is a module which detects and recognizes learned pictures, like pages of a comic books, faces of objects or even locations.
    ///
    /// The learning stage is done using the Choregraphe interface. Follow the steps in the green doc that will explain how to create your own database.
    ///
    /// The output value is written in ALMemory in the PictureDetected variable.
    ///
    /// It contains an array of tags, with the following format: 
    ///
    ///  
    ///
    /// [ [ TimeStampField ] [ Picture_info_0 , Picture _info_1, . . . , Picture_info_N-1 ] ] 
    ///
    ///  
    ///
    /// with as many Picture_info tags as things currently recognized. 
    ///
    /// Picture_info = [[labels_list], matched_keypoints, ratio, [boundary_points]] 
    ///
    /// with labels_list = [label_0, label_1, ..., label_N-1] and label_n belongs to label_n+1 
    ///
    /// and boundary_points = [[x0,y0], [x1,y1], ..., [xN,yN]] 
    ///
    ///  
    ///
    /// - Labels are the names given to the picture (e.g. \"cover/my book\", or \"fridge corner/kitchen/my flat\"). 
    ///
    /// - matched_keypoints corresponds to the number of keypoints retrieved in the current frame. 
    ///
    /// - ratio represents the number of keypoints found for the object in the current frame divided by the number of keypoints found during the learning stage. 
    ///
    /// - boundary_points is a list of points coordinates in angle values representing the reprojection in the current image of the boundaries selected during the learning stage.
  /// ALVisionRecognitionPostHandler gives acces to all bound methods of the module, using a thread around each call.
  /// Long running tasks can be stopped by calling stop(id) with the id returned by the post.
  /// </summary>
  ALVisionRecognitionProxyPostHandler();

  /// <summary>
  /// Exits and unregisters the module.
  /// </summary>
  /// <returns> brokerTaskID : The ID of the task assigned to it by the broker.</returns>
  int exit();
  /// <summary>
  /// Deprecated since 1.12
  /// </summary>
  /// <returns> brokerTaskID : The ID of the task assigned to it by the broker.</returns>
  int load();
  /// <summary>
  /// Set vision recognition parameters.
  /// </summary>
  /// <param name="paramName"> Name of the parameter to be changed. Among (\"resolution\", \"scoreThreshold\") </param>
  /// <param name="paramValue"> Value(s) of the parameter to be changed. </param>
  /// <returns> brokerTaskID : The ID of the task assigned to it by the broker.</returns>
  int setParam(const std::string& paramName, const AL::ALValue& paramValue);
  /// <summary>
  /// returns true if the method is currently running
  /// </summary>
  /// <param name="id"> the ID of the method to wait for </param>
  /// <returns> brokerTaskID : The ID of the task assigned to it by the broker.</returns>
  int stop(const int& id);
  /// <summary>
  /// Subscribes to the extractor. This causes the extractor to start writing information to memory using the keys described by getOutputNames(). These can be accessed in memory using ALMemory.getData(\"keyName\"). In many cases you can avoid calling subscribe on the extractor by just calling ALMemory.subscribeToEvent() supplying a callback method. This will automatically subscribe to the extractor for you.
  /// </summary>
  /// <param name="name"> Name of the module which subscribes. </param>
  /// <param name="period"> Refresh period (in milliseconds) if relevant. </param>
  /// <param name="precision"> Precision of the extractor if relevant. </param>
  /// <returns> brokerTaskID : The ID of the task assigned to it by the broker.</returns>
  int subscribe(const std::string& name, const int& period, const float& precision);
  /// <summary>
  /// Subscribes to the extractor. This causes the extractor to start writing information to memory using the keys described by getOutputNames(). These can be accessed in memory using ALMemory.getData(\"keyName\"). In many cases you can avoid calling subscribe on the extractor by just calling ALMemory.subscribeToEvent() supplying a callback method. This will automatically subscribe to the extractor for you.
  /// </summary>
  /// <param name="name"> Name of the module which subscribes. </param>
  /// <returns> brokerTaskID : The ID of the task assigned to it by the broker.</returns>
  int subscribe(const std::string& name);
  /// <summary>
  /// Unsubscribes from the extractor.
  /// </summary>
  /// <param name="name"> Name of the module which had subscribed. </param>
  /// <returns> brokerTaskID : The ID of the task assigned to it by the broker.</returns>
  int unsubscribe(const std::string& name);
  /// <summary>
  /// Updates the period if relevant.
  /// </summary>
  /// <param name="name"> Name of the module which has subscribed. </param>
  /// <param name="period"> Refresh period (in milliseconds). </param>
  /// <returns> brokerTaskID : The ID of the task assigned to it by the broker.</returns>
  int updatePeriod(const std::string& name, const int& period);
  /// <summary>
  /// Updates the precision if relevant.
  /// </summary>
  /// <param name="name"> Name of the module which has subscribed. </param>
  /// <param name="precision"> Precision of the extractor. </param>
  /// <returns> brokerTaskID : The ID of the task assigned to it by the broker.</returns>
  int updatePrecision(const std::string& name, const float& precision);
};

}
#endif // ALVISIONRECOGNITIONPROXYPOSTHANDLER_H_

