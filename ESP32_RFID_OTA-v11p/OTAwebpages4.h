//******* Web OTA Update Pages *****
//From https://randomnerdtutorials.com/esp32-over-the-air-ota-programming/  2022-12-26

/*
 * Login page:  NOTE EMBEDDED NAME/PASSWORD on Line 42
 */

const char* loginIndex =
 "<form name='loginForm'>"
    "<table width='30%' bgcolor='44BDB9' align='left'>"
        "<tr>"
            "<td colspan=2>"
                "<center><font size=4><b>ESP32-" 
                PCONFIG
                " OTA Login</b></font></center>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
             "<td>Username:</td>"
             "<td><input type='text' size=25 name='userid'><br></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td>Password:</td>"
            "<td><input type='Password' size=25 name='pwd'><br></td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td colspan=2>"
              "<center><input type='submit' onclick='check(this.form)' value='Login'></center>"
              "<br>"
            "</td>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=="
    "'XXX'"
    " && form.pwd.value=="
    "'XXX'"
    "){"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
"</script>";

/*
 * Server Index Page
 */

const char* serverIndex =
"<script src='https://code.jquery.com/jquery-3.6.3.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
    "<table width='30%' bgcolor='48BD44' align='left'>"
        "<tr>"
            "<td colspan=2 style='text-align:center;font-size:160%;'>"
                "<b>Bin File Upload</b>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
             "<td style='text-align:center;'>"
               "<input type='file' name='update'>"
               "<br>"
             "</td>"
             "<td></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td colspan=2 style='text-align:center;'>"
               "<input type='submit' value='Upload'>"
             "</td>"
            "<br>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td colspan=2 style='text-align:center;' id='prg'>"
              "UPLOAD PROGRESS: 0%"
            "</td>"
        "</tr>"
    "</table>"
 "</form>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('UPLOAD PROGRESS: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('SUCCESS!')"
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";